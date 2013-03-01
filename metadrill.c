// install libsdl1.2-dev libsdl-ttf2.0-dev
// gcc -o metadrill -ggdb -Wall -O0 metadrill.c -lm -lSDL -lSDL_ttf

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include <SDL/SDL.h>
#include <SDL/SDL_ttf.h>

#ifdef WIN32
#  include <io.h>
#  include <windows.h>
#else
#  include <termios.h>
#endif

#define Z_VALUE_UP (cnc_z)
#define Z_VALUE_MID (cnc_z-3)
#define Z_VALUE_DOWN (cnc_z-7)

#define FEEDRATE_HIGH 400
#define FEEDRATE_LOW 90

#ifdef WIN32
#  define TTS_FOR_GCODE "COM3"
#else
#  define TTS_FOR_GCODE "/dev/ttyUSB0"
#endif

#define CONSIZE 45
int console_gui = 0;
char conbuffer[CONSIZE][128] = { };
int conbuffer_i = 0;

#define console(fmt, ...) do { printf(fmt, ##__VA_ARGS__); fflush(stdout); \
  if (console_gui) { snprintf(conbuffer[conbuffer_i], 128, "%s" fmt, conbuffer[conbuffer_i], ##__VA_ARGS__); \
  int len = strlen(conbuffer[conbuffer_i]); screen_needs_update = 1; \
  if (len > 0 && conbuffer[conbuffer_i][len-1] == '\n') { conbuffer_i = (conbuffer_i+1) % CONSIZE; \
  conbuffer[conbuffer_i][0] = 0; } } } while (0)

// This is to not confuse the VIM syntax highlighting
#define CHECK_VAL_OPEN (
#define CHECK_VAL_CLOSE )

#define CHECK(result, check)                                          \
  CHECK_VAL_OPEN{                                                     \
    typeof(result) _R = (result);                                     \
    if (!(_R check)) {                                                \
      console("Error from '%s' (%d %s) in %s:%d.\n",                  \
                      #result, (int)_R, #check, __FILE__, __LINE__);  \
      console("ERRNO(%d): %s\n", errno, strerror(errno));             \
      abort();                                                        \
    }                                                                 \
    _R;                                                               \
  }CHECK_VAL_CLOSE

const char *tts_device = TTS_FOR_GCODE;

float min_x, max_x;
float min_y, max_y;

struct pos {
	struct pos *next;
	float x, y;
	int done;
};

struct pos *drill_list = NULL;
struct pos *mark_list = NULL;
struct pos *mount_list = NULL;

int mark_count = 0;
int mount_count = 0;
int drill_count = 0;

struct matrixop {
	float a, b, c, d, e, f;
};

struct transform_job {
	// output:
	float xp, yp;
	// input:
	float xf, yf;
	struct matrixop op;
};

struct adjust_job {
	// input:
	float xp[3], yp[3];
	float xf[3], yf[3];
	// output:
	struct matrixop op;
};

struct matrixop active_matrixop;

struct adjust_sample {
	struct adjust_sample *next;
	float xf, yf, xp, yp;
};

struct adjust_sample *adj_list = NULL;
int adj_count = 0;

#define Z_STATE_HOME -1
#define Z_STATE_SETHOME -2

#define Z_STATE_UP 0
#define Z_STATE_MID 1
#define Z_STATE_DOWN 2

float cnc_x, cnc_y, cnc_z;
float target_x, target_y;
float current_x, current_y;
int current_z, current_autopos;
int drilling, drilling_ok;
int blind_gcode_mode;

float manual_step_size;
int manual_step_index;

SDL_Surface *screen;
TTF_Font *font, *tiny_font;

int screen_needs_update = 1;

int get_morton_num(int v1, int v2);
int compare_pos_by_morton_num(const void *a_vp, const void *b_vp);
void read_drlfile(FILE *f);
void set_default_matrixop(struct matrixop *op);
void print_matrixop(struct matrixop *op);
void transform(struct transform_job *job);
void adjust(struct adjust_job *job);
int get_screen_x(float x);
int get_screen_y(float y);
void setpixel(int x, int y, int r, int g, int b);
void draw_screen();
void draw_move_line(float x1f, float y1f, float x2f, float y2f);
void move_cnc_head(float x, float y, int z);

int get_morton_num(int v1, int v2)
{
	int i, retval = 0;

	int bit(int v, int in_pos, int out_pos) {
		return ((v >> in_pos) & 1) << out_pos;
	}

	for (i=0; i<16; i++) {
		retval |= bit(v1, i, 2*i);
		retval |= bit(v2, i, 2*i+1);
	}

	return retval;
}

int compare_pos_by_morton_num(const void *a_vp, const void *b_vp)
{
	const struct pos *const *a = a_vp;
	const struct pos *const *b = b_vp;
	int a_mn = get_morton_num(get_screen_x((*a)->x), get_screen_y((*a)->y));
	int b_mn = get_morton_num(get_screen_x((*b)->x), get_screen_y((*b)->y));
	if (a_mn < b_mn)
		return -1;
	if (a_mn > b_mn)
		return +1;
	return 0;
}

void sort_drill_list_by_morton_num()
{
	struct pos **dl = malloc(sizeof(struct pos*)*drill_count);
	struct pos *p;
	int i;

	for (i=0, p=drill_list; p; i++, p=p->next)
		dl[i] = p;

	qsort(dl, drill_count, sizeof(struct pos*), &compare_pos_by_morton_num);

	drill_list = NULL;
	for (i=0; i < drill_count; i++) {
		dl[i]->next = drill_list;
		drill_list = dl[i];
	}

	free(dl);
}

void read_drlfile(FILE *f)
{
	char buf[512];
	int firstdrill = 1;
	struct pos **current_list = NULL;
	int *current_count = NULL;

	fgets(buf, 512, f);
	while (buf != NULL || buf != EOF) {
		printf("%s\n", buf);
		char s1[512], s2[512];
		float v1, v2;
		if (sscanf(buf, "T%*dC%f", &v1) >= 1) {
/*		FIXME use all the T value as drill
			current_list = NULL;
			current_count = NULL;
			if (v1 >= 0.004 && v1 <= 0.006) {
				current_list = &mark_list;
				current_count = &mark_count;
			}
			if (v1 >= 0.009 && v1 <= 2) {
				current_list = &mount_list;
				current_count = &mount_count;
			}
			if (v1 >= 2.001 && v1 <= 10) {*/
				current_list = &drill_list;
				current_count = &drill_count;
			//}
		} //end if  */
		if (sscanf(buf, "X%[-0-9]Y%[-0-9]", s1, s2) == 2) {
			sscanf(s1, "%f", &v1);
			sscanf(s2, "%f", &v2);
			printf("%f %f\n", v1, v2);
			v1 *= pow(10, 10-strlen(s1)) * (s1[0] == '-' ? 10 : 1);
			v2 *= pow(10, 10-strlen(s2)) * (s2[0] == '-' ? 10 : 1);
			if (firstdrill) {
				min_x = max_x = v1;
				min_y = max_y = v2;
				firstdrill = 0;
			}
			if (v1 < min_x)
				min_x = v1;
			if (v1 > max_x)
				max_x = v1;
			if (v2 < min_y)
				min_y = v2;
			if (v2 > max_y)
				max_y = v2;
			struct pos *p = malloc(sizeof(struct pos));
			p->x = v1;
			p->y = v2;
			p->done = 0;
			p->next = *current_list;
			*current_list = p;
			(*current_count)++;
		} //end if */
		int err = fgets(buf, 512, f);
		if (!err)
			break; 
	} //end while
	sort_drill_list_by_morton_num();
	console("Drillfile statistics:\n");
	console("     %5d mark positions\n", mark_count);
	console("     %5d mount positions\n", mount_count);
	console("     %5d drill positions\n", drill_count);
	console("     x-range: %f - %f\n", min_x, max_x);
	console("     y-range: %f - %f\n", min_y, max_y);
	current_x = min_x;
	current_y = min_y;
	current_z = Z_STATE_UP;
	target_x = max_x;
	target_y = max_y;
	drilling = 0;
}

void set_default_matrixop(struct matrixop *op)
{
	op->a = 1;
	op->b = 0;
	op->c = 0;
	op->d = 1;
	op->e = 0;
	op->f = 0;
}

void print_matrixop(struct matrixop *op)
{
	console("                    / %+e %+e \\\n", op->a, op->b);
	console("(xp yp) = (xf yf) * |                             | + (%+e %+e)\n", op->e, op->f);
	console("                    \\ %+e %+e /\n", op->c, op->d);
}

void transform(struct transform_job *job)
{
	/*
	 *                      / a b \
	 *  (xp yp) = (xf yf) * |     | + (e f)
	 *                      \ c d /
	 */
	job->xp = job->op.a * job->xf + job->op.c * job->yf + job->op.e;
	job->yp = job->op.b * job->xf + job->op.d * job->yf + job->op.f;
}

void adjust(struct adjust_job *job)
{
	/*** Maxima commands used: ***
		eqx1: xp1 = a*xf1 + c*yf1 + e;
		eqx2: xp2 = a*xf2 + c*yf2 + e;
		eqx3: xp3 = a*xf3 + c*yf3 + e;
		eqy1: yp1 = b*xf1 + d*yf1 + f;
		eqy2: yp2 = b*xf2 + d*yf2 + f;
		eqy3: yp3 = b*xf3 + d*yf3 + f;
		string(algsys([eqx1, eqx2, eqx3, eqy1, eqy2, eqy3],
				[a, b, c, d, e, f]));
	*****************************/
	float xp1 = job->xp[0], xp2 = job->xp[1], xp3 = job->xp[2];
	float yp1 = job->yp[0], yp2 = job->yp[1], yp3 = job->yp[2];
	float xf1 = job->xf[0], xf2 = job->xf[1], xf3 = job->xf[2];
	float yf1 = job->yf[0], yf2 = job->yf[1], yf3 = job->yf[2];
	job->op.a = ((xp2-xp1)*yf3+(xp1-xp3)*yf2+(xp3-xp2)*yf1)/((xf2-xf1)*yf3+(xf1-xf3)*yf2+(xf3-xf2)*yf1);
	job->op.b = -((yf2-yf1)*yp3+(yf1-yf3)*yp2+(yf3-yf2)*yp1)/((xf2-xf1)*yf3+(xf1-xf3)*yf2+(xf3-xf2)*yf1);
	job->op.c = ((xf2-xf1)*xp3+(xf1-xf3)*xp2+(xf3-xf2)*xp1)/((xf2-xf1)*yf3+(xf1-xf3)*yf2+(xf3-xf2)*yf1);
	job->op.d = ((xf2-xf1)*yp3+(xf1-xf3)*yp2+(xf3-xf2)*yp1)/((xf2-xf1)*yf3+(xf1-xf3)*yf2+(xf3-xf2)*yf1);
	job->op.e = -((xf1*xp2-xf2*xp1)*yf3+(xf3*xp1-xf1*xp3)*yf2+(xf2*xp3-xf3*xp2)*yf1)/((xf2-xf1)*yf3+(xf1-xf3)*yf2+(xf3-xf2)*yf1);
	job->op.f = ((xf1*yf2-xf2*yf1)*yp3+(xf3*yf1-xf1*yf3)*yp2+(xf2*yf3-xf3*yf2)*yp1)/((xf2-xf1)*yf3+(xf1-xf3)*yf2+(xf3-xf2)*yf1);
}

void adjust_run()
{
	struct adjust_sample *a1, *a2, *a3;
	struct matrixop resultop = { };
	int resultop_div = 0;

	if (adj_count < 3) {
		console("Need at least 3 points to run adjust!\n");
		return;
	}

	console("Adjust:\n");
	for (a1=adj_list; a1; a1=a1->next)
		console("[ %f %f ] => [ %f %f ]\n", a1->xf, a1->yf, a1->xp, a1->yp);

	for (a1=adj_list; a1; a1=a1->next)
	for (a2=adj_list; a2; a2=a2->next)
	for (a3=adj_list; a3; a3=a3->next)
	{
		if (a1 == a2 || a1 == a3 || a2 == a3)
			continue;

		struct adjust_job job;

		job.xp[0] = a1->xp;
		job.yp[0] = a1->yp;
		job.xf[0] = a1->xf;
		job.yf[0] = a1->yf;

		job.xp[1] = a2->xp;
		job.yp[1] = a2->yp;
		job.xf[1] = a2->xf;
		job.yf[1] = a2->yf;

		job.xp[2] = a3->xp;
		job.yp[2] = a3->yp;
		job.xf[2] = a3->xf;
		job.yf[2] = a3->yf;

		adjust(&job);

		resultop.a += job.op.a;
		resultop.b += job.op.b;
		resultop.c += job.op.c;
		resultop.d += job.op.d;
		resultop.e += job.op.e;
		resultop.f += job.op.f;

		resultop_div++;
	}

	resultop.a /= resultop_div;
	resultop.b /= resultop_div;
	resultop.c /= resultop_div;
	resultop.d /= resultop_div;
	resultop.e /= resultop_div;
	resultop.f /= resultop_div;

	active_matrixop = resultop;

	console("New matrices:\n");
	print_matrixop(&active_matrixop);

	for (a1=adj_list; a1; a1=a2) {
		a2 = a1->next;
		free(a1);
	}

	adj_list = NULL;
	adj_count = 0;
}

int get_screen_x(float x)
{
	return 620 - ((x-min_x)/(max_x-min_x))*620 + 10;
}

int get_screen_y(float y)
{
	return 450 - ((y-min_y)/(max_y-min_y))*440;
}

void setpixel(int x, int y, int r, int g, int b)
{
	Uint32 *pixel = screen->pixels;
	pixel[x + y*640] = r << 16 | g << 8 | b;
}

void draw_text(int x, int w, int y, TTF_Font *f, SDL_Color color, const char* text)
{
	SDL_Surface *text_surface;
	text_surface = CHECK(TTF_RenderText_Solid(f, text, color), != NULL);
	SDL_Rect drect = text_surface->clip_rect;
	drect.x = x; drect.y = y;
	if (w && w > drect.w)
		drect.x += (w-drect.w)/2;
	SDL_BlitSurface(text_surface, NULL, screen, &drect);
	SDL_FreeSurface(text_surface);
}

void draw_screen()
{
	int x, y, i, j;

	SDL_Color textcolor1 = {64, 64, 64};
	SDL_Color textcolor2 = {255, 255, 255};

	if (!screen_needs_update)
		return;
	screen_needs_update = 0;

	if (drilling) {
		Uint32 *pixel = screen->pixels;
		for (i=0; i<640*480; i++) {
			*pixel = 0x00880000;
			pixel++;
		}
	} else
		memset(screen->pixels, 0, 640*480*4);

	if (console_gui) {
		for (i=1; i<=CONSIZE; i++) {
			char *str = conbuffer[(conbuffer_i+i) % CONSIZE];
			int len = strlen(str);
			while (len > 0 && str[len-1] == '\n')
				str[--len] = 0;
			if (len > 0)
				draw_text(0, 0, i*10, tiny_font, textcolor1, str);
		}
	}

	struct pos *p;

	for (p=drill_list; p; p=p->next) {
		if (p->done)
			setpixel(get_screen_x(p->x), get_screen_y(p->y), 0x88, 0x88, 0x88);
		else
			setpixel(get_screen_x(p->x), get_screen_y(p->y), 0xff, 0xff, 0xff);
	}
	for (p=mark_list; p; p=p->next)
		setpixel(get_screen_x(p->x), get_screen_y(p->y), 0x00, 0xff, 0xff);
	for (p=mount_list; p; p=p->next)
		setpixel(get_screen_x(p->x), get_screen_y(p->y), 0xff, 0x88, 0x00);

	static const char *current_cursor[3][13] = {
		{
			"      #      ",
			"     ###     ",
			"    #####    ",
			"   ### ###   ",
			"  ###   ###  ",
			" ###     ### ",
			"###       ###",
			" ###     ### ",
			"  ###   ###  ",
			"   ### ###   ",
			"    #####    ",
			"     ###     ",
			"      #      ",
		},
		{
			"             ",
			"             ",
			"  #       #  ",
			"   #     #   ",
			"    #####    ",
			"    #   #    ",
			"    #   #    ",
			"    #   #    ",
			"    #####    ",
			"   #     #   ",
			"  #       #  ",
			"             ",
			"             ",
		},
		{
			"             ",
			"             ",
			"             ",
			"   #     #   ",
			"    #   #    ",
			"     # #     ",
			"             ",
			"     # #     ",
			"    #   #    ",
			"   #     #   ",
			"             ",
			"             ",
			"             ",
		},
	};

	if (current_autopos) {
		x = get_screen_x(current_x)-6;
		y = get_screen_y(current_y)-6;
		for (i=0; i<13; i++)
		for (j=0; j<13; j++)
			if (current_cursor[current_z][j][i] != ' ')
				setpixel(x+i, y+j, 0xff, 0x00, 0xff);
	}

	static const char *target_cursor[] = {
		"  ###  ",
		"   #   ",
		"#     #",
		"##   ##",
		"#     #",
		"   #   ",
		"  ###  ",
	};

	x = get_screen_x(target_x)-3;
	y = get_screen_y(target_y)-3;
	for (i=0; i<7; i++)
	for (j=0; j<7; j++)
		if (target_cursor[j][i] != ' ')
			setpixel(x+i, y+j, 0x00, 0xff, 0xff);

	static const char *adj_marker[] = {
		"  ###  ",
		" #   # ",
		"#     #",
		"#     #",
		"#     #",
		" #   # ",
		"  ###  ",
	};

	struct adjust_sample *adj;
	for (adj=adj_list; adj; adj=adj->next) {
		x = get_screen_x(adj->xf)-3;
		y = get_screen_y(adj->yf)-3;
		for (i=0; i<7; i++)
		for (j=0; j<7; j++)
			if (adj_marker[j][i] != ' ')
				setpixel(x+i, y+j, 0x00, 0xff, 0xff);
	}

	char strbuf[512];
	snprintf(strbuf, 512, "M-Step: %f (%d), CNC-X: %f, CNC-Y: %f",
			manual_step_size, manual_step_index, cnc_x, cnc_y);
	draw_text(0, 0, 460, font, textcolor2, strbuf);

	SDL_UpdateRect(screen, 0, 0, 640, 480);
}

void draw_move_line(float x1f, float y1f, float x2f, float y2f)
{
	int x1 = get_screen_x(x1f);
	int y1 = get_screen_y(y1f);
	int x2 = get_screen_x(x2f);
	int y2 = get_screen_y(y2f);
	float xd = x2-x1, yd = y2-y1;
	int steps, i;

	if (abs(x1-x2) > abs(y1-y2))
		steps = abs(x1-x2)*2;
	else
		steps = abs(y1-y2)*2;

	float x=x1, y=y1;
	for (i=0; i<steps; i++) {
		setpixel(x, y, 0xff, 0x00, 0xff);
		x+=xd/steps;
		y+=yd/steps;
	}
	
	SDL_UpdateRect(screen, 0, 0, 640, 480);
}

void move_cnc_head_gcode(int z_state, int z_notxy, int low_speed)
{
	static int initialized = 0;
#ifdef WIN32
	static HANDLE hComm;
#else
	static FILE *tts = NULL;
#endif
	char buffer[514];

	void execute_gcode()
	{
		int len = strlen(buffer);
		console("Sending GCODE (len=%d): %s\n", len+2, buffer);
		// buffer[len++] = '\r';
		buffer[len++] = '\n';
		buffer[len] = 0;
		
#ifdef WIN32
		int written = 0;
		while (written < len) {
			DWORD wr_ret;
			CHECK(WriteFile(hComm, buffer+written, len-written, &wr_ret, NULL), != 0);
			CHECK(wr_ret, >0);
			written += wr_ret;
		}
#else
		fprintf(tts, "%s", buffer);
		fflush(tts);

		if (blind_gcode_mode)
			return;
#endif
read_next_line:
		console("Answer from CNC: ");
		draw_screen();
#ifdef WIN32
		len = 0;
		do {
			DWORD rd_ret;
			int update_screen_text = 1;
retry_read_file:
			CHECK(ReadFile(hComm, buffer+len, 1, &rd_ret, NULL), != 0);
			if (rd_ret == 0) {
				if (update_screen_text)
					draw_screen();
				update_screen_text = 0;
				goto retry_read_file;
			}
			CHECK(rd_ret, >0);
			if (buffer[len] == '\r')
				console("\\r");
			else if (buffer[len] == '\n')
				console("\\n");
			else if (buffer[len] < ' ')
				console("\\%03o", (unsigned char)buffer[len]);
			else
				console("%c", buffer[len]);
			buffer[++len] = 0;
		} while (buffer[len-1] != '\n' && len < 500);
		console("\n");
		draw_screen();
#else
		CHECK(fgets(buffer, 512, tts), == buffer);
		console("%s", buffer);
#endif
		if(strcmp(buffer, "ok\r\n") && strncmp(buffer, "ok:", 3)) {
			console("That isn't what was expected. (reading next line)\n");
			goto read_next_line;
		}
	}

	if (!initialized)
	{
#ifdef WIN32
		hComm = CHECK(CreateFile(tts_device, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0), != INVALID_HANDLE_VALUE);

		DCB dcb;
		FillMemory(&dcb, sizeof(dcb), 0);
		dcb.DCBlength = sizeof(dcb);
		CHECK(BuildCommDCB("38400,n,8,1", &dcb), != 0);
		CHECK(SetCommState(hComm, &dcb), != 0);

		COMMTIMEOUTS ct;
		FillMemory(&ct, sizeof(ct), 0);
		ct.ReadIntervalTimeout = MAXDWORD;
		ct.ReadTotalTimeoutMultiplier = MAXDWORD;
		ct.ReadTotalTimeoutConstant = 100;
		CHECK(SetCommTimeouts(hComm, &ct), != 0);
#else
		if (blind_gcode_mode) {
			tts = CHECK(fopen(tts_device, "w"), != NULL);
		} else {
			tts = CHECK(fopen(tts_device, "r+"), != NULL);

			int comfd = fileno(tts);
			struct termios newtio = { };
			newtio.c_cflag = B38400 | CS8 | CREAD;
			newtio.c_iflag = IGNPAR;
			newtio.c_oflag = 0;
			newtio.c_lflag = 0;
			newtio.c_cc[VMIN]=1;
			newtio.c_cc[VTIME]=0;
			tcflush(comfd, TCIFLUSH);
			tcsetattr(comfd, TCSANOW, &newtio);
		}
#endif

		snprintf(buffer, 512, "G90");
		execute_gcode();
		snprintf(buffer, 512, "G92");
		execute_gcode();

		initialized = 1;
	}

	if (z_state == Z_STATE_HOME) {
#if 1
		snprintf(buffer, 512, "G1 Z0 F%f",
				low_speed ? (float)FEEDRATE_LOW : (float)FEEDRATE_HIGH);
		execute_gcode();
		snprintf(buffer, 512, "G1 X0 Y0 F%f",
				low_speed ? (float)FEEDRATE_LOW : (float)FEEDRATE_HIGH);
		execute_gcode();
#else
		snprintf(buffer, 512, "G90");
		execute_gcode();
		snprintf(buffer, 512, "G30 Y0 X0 Z0 F%f", (float)FEEDRATE_HIGH);
		execute_gcode();
#endif
		cnc_x = cnc_y = cnc_z = 0;
		current_z = 0;
		return;
	}

	if (z_state == Z_STATE_SETHOME) {
		snprintf(buffer, 512, "G92 X%f Y%f Z%d", current_x, current_y, current_z);
		execute_gcode();
		cnc_x = cnc_y = cnc_z = 0;
		current_z = 0;
		return;
	}

	float z = Z_VALUE_UP;
	if (z_state == Z_STATE_UP)
		z = Z_VALUE_UP;
	if (z_state == Z_STATE_MID)
		z = Z_VALUE_MID;
	if (z_state == Z_STATE_DOWN) {
		if (drilling_ok && current_autopos) {
			z = Z_VALUE_DOWN;
		} else {
			console("Drilling is not possible at the moment!\n");
			console("(Start app with -x and use auto positioning.)\n");
			z = Z_VALUE_MID;
		}
	}

	if (z_notxy) {
		snprintf(buffer, 512, "G1 Z%f F%f", z,
				low_speed ? (float)FEEDRATE_LOW : (float)FEEDRATE_HIGH);
	} else {
		snprintf(buffer, 512, "G1 X%f Y%f F%f", cnc_x, cnc_y,
				low_speed ? (float)FEEDRATE_LOW : (float)FEEDRATE_HIGH);
	}
	execute_gcode();
	current_z = z_state;
}

void move_cnc_head_setpos(float x, float y)
{
	struct transform_job tj = { };
	tj.xf = x;
	tj.yf = y;
	tj.op = active_matrixop;
	transform(&tj);
	cnc_x = tj.xp;
	cnc_y = tj.yp;
}

void move_cnc_head_rel(float xd, float yd, float zd)
{
	if (fabs(xd) > 10 || fabs(yd) > 10 || current_z == Z_STATE_UP) {
		console("Relative move (fast): X_delta=%f, Y_delta=%f\n", xd, yd);
		cnc_z += zd;
		move_cnc_head_gcode(Z_STATE_UP, 1, 0);
		cnc_x += xd;
		cnc_y += yd;
		move_cnc_head_gcode(Z_STATE_UP, 0, 0);
	} else {
		console("Relative move (slow): X_delta=%f, Y_delta=%f\n", xd, yd);
		cnc_z += zd;
		move_cnc_head_gcode(Z_STATE_MID, 1, 0);
		cnc_x += xd;
		cnc_y += yd;
		move_cnc_head_gcode(Z_STATE_MID, 0, 1);
	}
	screen_needs_update = 1;
	draw_screen();
}

void move_cnc_head(float x, float y, int z)
{
	if ((x != current_x || y != current_y || z == Z_STATE_UP) && current_z != Z_STATE_UP)
	{
		console("Moving head up.\n");
		move_cnc_head_gcode(Z_STATE_UP, 1, 0);
		screen_needs_update = 1;
		draw_screen();
	}

	console("Moving head to X=%f, Y=%f.\n", x, y);
	draw_move_line(current_x, current_y, x, y);
	move_cnc_head_setpos(x, y);
	move_cnc_head_gcode(Z_STATE_UP, 0, 0);
	move_cnc_head_gcode(Z_STATE_UP, 1, 0);
	screen_needs_update = 1;
	current_x = x;
	current_y = y;
	draw_screen();

	if (z == Z_STATE_DOWN) {
		console("Moving head down to drill position.\n");
		move_cnc_head_gcode(Z_STATE_MID, 1, 0);
		move_cnc_head_gcode(z, 1, 1);
		screen_needs_update = 1;
		draw_screen();
	}

	if (z == Z_STATE_MID || z == Z_STATE_DOWN) {
		console("Moving head down to mid position.\n");
		move_cnc_head_gcode(Z_STATE_MID, 1, 0);
		screen_needs_update = 1;
		draw_screen();
	}
}

int main(int argc, char **argv)
{
	while (1) {
		if (argc > 1 && !strcmp(argv[1], "-x")) {
			argc--; argv++;
			drilling_ok = 1;
			continue;
		}
		if (argc > 1 && !strcmp(argv[1], "-c")) {
			argc--; argv++;
			console_gui = 1;
			continue;
		}
#ifndef WIN32
		if (argc > 1 && !strcmp(argv[1], "-g")) {
			argc--; argv++;
			blind_gcode_mode = 1;
			continue;
		}
		break;
#endif
	}

	CHECK(argc, == 2 || _R == 3);

	if (argc == 3)
		tts_device = argv[2];

	CHECK(SDL_Init(SDL_INIT_VIDEO), >= 0);
	SDL_WM_SetCaption("Metadrill", "Metadrill");
	atexit(SDL_Quit);

	CHECK(TTF_Init(), >= 0);

	screen = CHECK(SDL_SetVideoMode(640, 480, 32, SDL_SWSURFACE), != NULL);
	font = CHECK(TTF_OpenFont("font.ttf", 16), != NULL);
	tiny_font = CHECK(TTF_OpenFont("font.ttf", 8), != NULL);

	console("Loaded transfomation matrices:\n");
	print_matrixop(&active_matrixop);
	cnc_z = 0;

	FILE *f = CHECK(fopen(argv[1], "r"), != NULL);
	read_drlfile(f);
	fclose(f);

	set_default_matrixop(&active_matrixop);
	if ((f = fopen("metadrill.mat", "r")) != NULL) {
		fscanf(f, "%f\n", &active_matrixop.a);
		fscanf(f, "%f\n", &active_matrixop.b);
		fscanf(f, "%f\n", &active_matrixop.c);
		fscanf(f, "%f\n", &active_matrixop.d);
		fscanf(f, "%f\n", &active_matrixop.e);
		fscanf(f, "%f\n", &active_matrixop.f);
		fclose(f);
	}

	while (1)
	{
		draw_screen();

                SDL_Event event;
		SDL_WaitEvent(NULL);
                while (!screen_needs_update && SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT)
				goto app_quit;
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_q)
				goto app_quit;
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_a)
			{
				adjust_run();
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_p)
			{
				struct adjust_sample *adj = malloc(sizeof(struct adjust_sample));
				adj->xf = target_x;
				adj->yf = target_y;
				adj->xp = cnc_x;
				adj->yp = cnc_y;
				adj->next = adj_list;
				adj_list = adj;
				adj_count++;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_w)
			{
				console("Writing new version of metadrill.mat.\n");
				f = CHECK(fopen("metadrill.mat", "w"), != NULL);
				fprintf(f, "%+e\n", active_matrixop.a);
				fprintf(f, "%+e\n", active_matrixop.b);
				fprintf(f, "%+e\n", active_matrixop.c);
				fprintf(f, "%+e\n", active_matrixop.d);
				fprintf(f, "%+e\n", active_matrixop.e);
				fprintf(f, "%+e\n", active_matrixop.f);
				fclose(f);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_0)
			{
				manual_step_size = 0;
				manual_step_index = 0;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_1)
			{
				manual_step_size = 100;
				manual_step_index = 1;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_2)
			{
				manual_step_size = 30;
				manual_step_index = 2;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_3)
			{
				manual_step_size = 10;
				manual_step_index = 3;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_4)
			{
				manual_step_size = 3;
				manual_step_index = 4;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_5)
			{
				manual_step_size = 1;
				manual_step_index = 5;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_6)
			{
				manual_step_size = 0.3;
				manual_step_index = 6;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_7)
			{
				manual_step_size = 0.1;
				manual_step_index = 7;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_8)
			{
				manual_step_size = 0.03;
				manual_step_index = 8;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_9)
			{
				manual_step_size = 0.01;
				manual_step_index = 9;
				screen_needs_update = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_LEFT)
			{
				current_autopos = 0;
				move_cnc_head_rel(-manual_step_size, 0, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_RIGHT)
			{
				current_autopos = 0;
				move_cnc_head_rel(+manual_step_size, 0, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_UP)
			{
				current_autopos = 0;
				move_cnc_head_rel(0, +manual_step_size, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_DOWN)
			{
				current_autopos = 0;
				move_cnc_head_rel(0, -manual_step_size, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_PAGEDOWN)
			{
				current_autopos = 0;
				move_cnc_head_rel(0, 0, -manual_step_size);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_PAGEUP)
			{
				current_autopos = 0;
				move_cnc_head_rel(0, 0, +manual_step_size);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_h)
			{
				current_autopos = 0;
				move_cnc_head_gcode(Z_STATE_HOME, 0, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_z)
			{
				current_autopos = 0;
				move_cnc_head_gcode(Z_STATE_UP, 1, 0);
				move_cnc_head_gcode(Z_STATE_SETHOME, 0, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_m)
			{
				current_autopos = 1;
				move_cnc_head(target_x, target_y, Z_STATE_UP);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_d)
			{
				move_cnc_head_gcode(Z_STATE_MID, 1, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_e)
			{
				//enable drill possibility
				drilling_ok = 1;
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_u)
			{
				move_cnc_head_gcode(Z_STATE_UP, 1, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_x)
			{
				move_cnc_head_gcode(Z_STATE_MID, 1, 0);
				move_cnc_head_gcode(Z_STATE_DOWN, 1, 1);
				move_cnc_head_gcode(Z_STATE_MID, 1, 0);
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_c)
			{
				if (!console_gui) {
					console_gui = 1;
				} else {
					int i;
					for (i=0; i<CONSIZE; i++)
						conbuffer[i][0] = 0;
					screen_needs_update = 1;
					draw_screen();
				}
			}
			if (event.type == SDL_KEYDOWN &&
					event.key.keysym.sym == SDLK_s)
			{
				struct pos *p;
				drilling = 1;
				screen_needs_update = 1;
				for (p=drill_list; p; p=p->next) {
					if (p->done)
						continue;
					while (SDL_PollEvent(&event)) {
						if (event.type == SDL_QUIT)
							goto app_quit;
						if (event.type == SDL_KEYDOWN)
							goto abort_drilling;
					}
					current_autopos = 1;
					target_x = p->x;
					target_y = p->y;
					screen_needs_update = 1;
					draw_screen();
					move_cnc_head(target_x, target_y, Z_STATE_DOWN);
					move_cnc_head(target_x, target_y, Z_STATE_UP);
					p->done = 1;
				}
abort_drilling:
				drilling = 0;
				screen_needs_update = 1;
			}
			if (event.type == SDL_MOUSEBUTTONDOWN &&
					event.button.button == SDL_BUTTON_LEFT)
			{
				int x = event.button.x;
				int y = event.button.y;
				float best_delta = 20;

				void checkpos(struct pos *p) {
					float dx = fabs(get_screen_x(p->x) - x);
					float dy = fabs(get_screen_y(p->y) - y);
					float delta = sqrt(dx*dx + dy*dy);
					if (delta < best_delta) {
						best_delta = delta;
						target_x = p->x;
						target_y = p->y;
						screen_needs_update = 1;
					}
				}

				struct pos *p;
				for (p=drill_list; p; p=p->next)
					checkpos(p);
				for (p=mark_list; p; p=p->next)
					checkpos(p);
				if (screen_needs_update)
					console("New target position: X=%f, Y=%f\n",
							target_x, target_y);
			}
		}
	}

app_quit:
	console("Bye.\n");
	return 0;
}

