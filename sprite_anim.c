
///////////////////////////////////////////////////////////////////////////////
// Headers.

#include <stdint.h>
#include "system.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <stdbool.h>

#include "sprites_rgb333.h"
#include "tanks_rgb333.h"
#include "tank_rgb333.h"
#include "bullet_rgb333.h"

///////////////////////////////////////////////////////////////////////////////
// HW stuff.

#define WAIT_UNITL_0(x) while(x != 0){}
#define WAIT_UNITL_1(x) while(x != 1){}

#define SCREEN_IDX1_W 640
#define SCREEN_IDX1_H 480
#define SCREEN_IDX4_W 320
#define SCREEN_IDX4_H 240
#define SCREEN_RGB333_W 480
#define SCREEN_RGB333_H 360

#define SCREEN_IDX4_W8 (SCREEN_IDX4_W/8)

#define gpu_p32 ((volatile uint32_t*)LPRS2_GPU_BASE)
#define palette_p32 ((volatile uint32_t*)(LPRS2_GPU_BASE+0x1000))
#define unpack_idx1_p32 ((volatile uint32_t*)(LPRS2_GPU_BASE+0x400000))
#define pack_idx1_p32 ((volatile uint32_t*)(LPRS2_GPU_BASE+0x600000))
#define unpack_idx4_p32 ((volatile uint32_t*)(LPRS2_GPU_BASE+0x800000))
#define pack_idx4_p32 ((volatile uint32_t*)(LPRS2_GPU_BASE+0xa00000))
#define unpack_rgb333_p32 ((volatile uint32_t*)(LPRS2_GPU_BASE+0xc00000))
#define joypad_p32 ((volatile uint32_t*)LPRS2_JOYPAD_BASE)

typedef struct {
	unsigned a      : 1;
	unsigned b      : 1;
	unsigned z      : 1;
	unsigned start  : 1;
	unsigned up     : 1;
	unsigned down   : 1;
	unsigned left   : 1;
	unsigned right  : 1;
} bf_joypad;
#define joypad (*((volatile bf_joypad*)LPRS2_JOYPAD_BASE))

typedef struct {
	uint32_t m[SCREEN_IDX1_H][SCREEN_IDX1_W];
} bf_unpack_idx1;
#define unpack_idx1 (*((volatile bf_unpack_idx1*)unpack_idx1_p32))

///////////////////////////////////////////////////////////////////////////////
// Game config.

#define STEP 1
#define PACMAN_ANIM_DELAY 3

///////////////////////////////////////////////////////////////////////////////
// Game data structures.

typedef struct {
	uint16_t x;
	uint16_t y;
} point_t;

typedef enum {
	PACMAN_IDLE,
	PACMAN_OPENING_MOUTH,
	PACMAN_WITH_OPEN_MOUTH,
	PACMAN_CLOSING_MOUTH,
	PACMAN_WITH_CLOSED_MOUTH
} pacman_anim_states_t;

typedef enum
{
	LEFT,
	RIGHT,
	UP,
	DOWN
} pacman_direction;

typedef struct {
	pacman_anim_states_t state;
	uint8_t delay_cnt;
} pacman_anim_t;

typedef struct {
	point_t pos;
	pacman_anim_t anim;
	pacman_direction dir;
} pacman_t;

typedef struct {
	pacman_t pacman;
} game_state_t;

void draw_sprite_from_atlas(
	uint16_t src_x,
	uint16_t src_y,
	uint16_t w,
	uint16_t h,
	uint16_t dst_x,
	uint16_t dst_y
) {
	for(uint16_t y = 0; y < h; y++){
		for(uint16_t x = 0; x < w; x++){
			uint32_t src_idx =
				(src_y+y)*Pacman_Sprite_Map__w +
				(src_x+x);
			uint32_t dst_idx =
				(dst_y+y)*SCREEN_RGB333_W +
				(dst_x+x);
			uint16_t pixel = Pacman_Sprite_Map__p[src_idx];
			if(pixel != 0)
				unpack_rgb333_p32[dst_idx] = pixel;
		}
	}
}

void draw_map_from_atlas(
	uint16_t src_x,
	uint16_t src_y,
	uint16_t w,
	uint16_t h,
	uint16_t dst_x,
	uint16_t dst_y
) {
	for(uint16_t y = 0; y < h; y++){
		for(uint16_t x = 0; x < w; x++){
			uint32_t src_idx =
				(src_y+y)*tanks__w +
				(src_x+x);
			uint32_t dst_idx =
				(dst_y+y)*SCREEN_RGB333_W +
				(dst_x+x);
			uint16_t pixel = tanks__p[src_idx];
			unpack_rgb333_p32[dst_idx] = pixel;
		}
	}
}

void draw_tank_from_atlas(
	uint16_t src_x,
	uint16_t src_y,
	uint16_t w,
	uint16_t h,
	uint16_t dst_x,
	uint16_t dst_y
) {
	for(uint16_t y = 0; y < h; y++){
		for(uint16_t x = 0; x < w; x++){
			uint32_t src_idx =
				(src_y+y)*tank__w +
				(src_x+x);
			uint32_t dst_idx =
				(dst_y+y)*SCREEN_RGB333_W +
				(dst_x+x);
			uint16_t pixel = tank__p[src_idx];
			if(pixel != 0)
				unpack_rgb333_p32[dst_idx] = pixel;
		}
	}
}

void draw_bullet_from_atlas(
	uint16_t src_x,
	uint16_t src_y,
	uint16_t w,
	uint16_t h,
	uint16_t dst_x,
	uint16_t dst_y
) {
	for(uint16_t y = 0; y < h; y++){
		for(uint16_t x = 0; x < w; x++){
			uint32_t src_idx =
				(src_y+y)*bullet__w +
				(src_x+x);
			uint32_t dst_idx =
				(dst_y+y)*SCREEN_RGB333_W +
				(dst_x+x);
			uint16_t pixel = bullet__p[src_idx];
			if(pixel != 0)
				unpack_rgb333_p32[dst_idx] = pixel;
		}
	}
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tcgetattr", errno);
		printf("Prvi je problem\n");
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
		printf("Drugi je problem\n");
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0);
                //error_message ("error %d setting term attributes", errno);
}

char movement;
char moveTank;

void* readDesc(void* par)
{
	char* port = "/dev/ttyUSB1";
	int desc = open(port, O_RDWR | O_NOCTTY | O_SYNC);
	if (desc < 0)
	{
		printf("Nije kreiran deskriptor za serijski port\n");
		return 0;
	}

	set_interface_attribs (desc, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (desc, 0);
	char b[1];
	while(1)
	{
		int p = read(desc, b, sizeof b);
		movement = b[0];
		memset(&b, 0, sizeof b);
		usleep(10);

	}

}

void* readDesc1(void* par)
{
	char* port = "/dev/ttyUSB0";
	int desc = open(port, O_RDWR | O_NOCTTY | O_SYNC);
	if (desc < 0)
	{
		printf("Nije kreiran deskriptor za serijski port\n");
		return 0;
	}

	set_interface_attribs (desc, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (desc, 0);
	char b[1];
	while(1)
	{
		int p = read(desc, b, sizeof b);
		moveTank = b[0];
		memset(&b, 0, sizeof b);
		usleep(10);

	}

}

#define walls 2

///////////////////////////////////////////////////////////////////////////////
// Game code.

int main(void) {

	int p = 0;
	pthread_t thread1;
	pthread_create(&thread1, NULL, readDesc, (void*)&p);
	pthread_t thread2;
	pthread_create(&thread2, NULL, readDesc1, (void*)&p);

	// Setup.
	gpu_p32[0] = 3; // RGB333 mode.
	gpu_p32[0x800] = 0x00ff00ff; // Magenta for HUD.
	srand(time(0));

	// Game state.
	game_state_t gs;

	game_state_t tank;
	game_state_t bullet;
	gs.pacman.pos.x = 0;
	gs.pacman.pos.y = 0;
	tank.pacman.pos.x = 192;
	tank.pacman.pos.y = 192;
	gs.pacman.anim.state = PACMAN_IDLE;
	gs.pacman.anim.delay_cnt = 0;
	gs.pacman.dir = RIGHT;

	int lowerX = 16;
	int upperX = SCREEN_RGB333_H - 32 ;
	int lowerY = 16;
	int upperY = SCREEN_RGB333_W - 32;

	int playerEaten;
	int movementEnable = 0;

	pacman_direction prev_direction;

	draw_map_from_atlas(0, 0, 208, 208, 0, 0);

	draw_tank_from_atlas(128, 0, 16, 16, tank.pacman.pos.x, tank.pacman.pos.y);
	
	srand(time(0));
	char b;
	char t;
	int cnt = 0;

	int dist;

	bullet.pacman.pos.x = 300;
	bullet.pacman.pos.y = 300;

	bool bul = false;
	int skip = 0;
	bool pacWon = false;
	bool tankWon = false;

	//511,64,365 BOJE TENKA
	//63 BOJA PAKIJA

	while(1){

		b = movement;
		t = moveTank;
		
		cnt++;

		int mov_x = 0;
		int mov_y = 0;
		int movTank_x = 0;
		int movTank_y = 0;

		if(movement == 'd')
		{
//511,64,365 BOJE TENKA
			if((gs.pacman.pos.x <= 208 - 16))
			{
				if(((unpack_rgb333_p32[(gs.pacman.pos.y+2)*SCREEN_RGB333_W + 15 + gs.pacman.pos.x] == 0) && (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + gs.pacman.pos.x] == 0)) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + gs.pacman.pos.x] == 511) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + gs.pacman.pos.x] == 64) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + gs.pacman.pos.x] == 365) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + gs.pacman.pos.x] == 511) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + gs.pacman.pos.x] == 64) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + gs.pacman.pos.x] == 365))
					mov_x = +1;
			}
			else
				mov_x = 0;
			gs.pacman.dir = RIGHT;
		}
		else if(movement == 'a')
		{
			if((gs.pacman.pos.x >= 1))
			{
				if(((unpack_rgb333_p32[(gs.pacman.pos.y+2)*SCREEN_RGB333_W + gs.pacman.pos.x] == 0) && (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + gs.pacman.pos.x] == 0)) || (unpack_rgb333_p32[(gs.pacman.pos.y+2)*SCREEN_RGB333_W + gs.pacman.pos.x] == 511) || (unpack_rgb333_p32[(gs.pacman.pos.y+2)*SCREEN_RGB333_W + gs.pacman.pos.x] == 64) || (unpack_rgb333_p32[(gs.pacman.pos.y+2)*SCREEN_RGB333_W + gs.pacman.pos.x] == 365) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + gs.pacman.pos.x] == 511) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + gs.pacman.pos.x] == 64) || (unpack_rgb333_p32[(gs.pacman.pos.y+13)*SCREEN_RGB333_W + gs.pacman.pos.x] == 365))
					mov_x = -1;
			}
			else
				mov_x = 0;
			gs.pacman.dir = LEFT;
		}
		else if(movement == 'w')
		{
			if(gs.pacman.pos.y >= 1)
			{
				if(((unpack_rgb333_p32[(gs.pacman.pos.y-1)*SCREEN_RGB333_W + gs.pacman.pos.x + 2] == 0) && 
					(unpack_rgb333_p32[(gs.pacman.pos.y-1)*SCREEN_RGB333_W + 13 + gs.pacman.pos.x] == 0)) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y-1)*SCREEN_RGB333_W + gs.pacman.pos.x + 2] == 511) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y-1)*SCREEN_RGB333_W + gs.pacman.pos.x + 2] == 64) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y-1)*SCREEN_RGB333_W + gs.pacman.pos.x + 2] == 365) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y-1)*SCREEN_RGB333_W + 13 + gs.pacman.pos.x] == 511) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y-1)*SCREEN_RGB333_W + 13 + gs.pacman.pos.x] == 64) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y-1)*SCREEN_RGB333_W + 13 + gs.pacman.pos.x] == 365))					

					mov_y = -1;
			}
			else
				mov_y = 0;
			gs.pacman.dir = UP;
		}
		else if(movement == 's')
		{
			if(gs.pacman.pos.y <= 208 - 16)
			{
				if(((unpack_rgb333_p32[(gs.pacman.pos.y+17)*SCREEN_RGB333_W + gs.pacman.pos.x + 2] == 0) &&
 					(unpack_rgb333_p32[(gs.pacman.pos.y+17)*SCREEN_RGB333_W + 13 + gs.pacman.pos.x] == 0)) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y+17)*SCREEN_RGB333_W + gs.pacman.pos.x + 2] == 511) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y+17)*SCREEN_RGB333_W + gs.pacman.pos.x + 2] == 64) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y+17)*SCREEN_RGB333_W + gs.pacman.pos.x + 2] == 365) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y+17)*SCREEN_RGB333_W + 13 + gs.pacman.pos.x] == 511) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y+17)*SCREEN_RGB333_W + 13 + gs.pacman.pos.x] == 64) ||
					(unpack_rgb333_p32[(gs.pacman.pos.y+17)*SCREEN_RGB333_W + 13 + gs.pacman.pos.x] == 365))

					mov_y = +1;
			}
			else
				mov_y = 0;
			gs.pacman.dir = DOWN;
		}

		if(moveTank == 'd')
		{
			if((tank.pacman.pos.x <= 208 - 16))
			{
				if(((unpack_rgb333_p32[(tank.pacman.pos.y+2)*SCREEN_RGB333_W + 15 + tank.pacman.pos.x] == 0) &&
 					(unpack_rgb333_p32[(tank.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + tank.pacman.pos.x] == 0)) ||
					(unpack_rgb333_p32[(tank.pacman.pos.y+2)*SCREEN_RGB333_W + 15 + tank.pacman.pos.x] == 63) ||
					(unpack_rgb333_p32[(tank.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + tank.pacman.pos.x] == 63))
					movTank_x = +1;
			}
			else
				movTank_x = 0;
			tank.pacman.dir = RIGHT;
		}
		else if(moveTank == 'a')
		{
			if((tank.pacman.pos.x >= 1))
			{
				if(((unpack_rgb333_p32[(tank.pacman.pos.y+2)*SCREEN_RGB333_W + tank.pacman.pos.x] == 0) &&
				 (unpack_rgb333_p32[(tank.pacman.pos.y+13)*SCREEN_RGB333_W + tank.pacman.pos.x] == 0)) ||
				 (unpack_rgb333_p32[(tank.pacman.pos.y+2)*SCREEN_RGB333_W + tank.pacman.pos.x] == 63) ||
				 (unpack_rgb333_p32[(tank.pacman.pos.y+13)*SCREEN_RGB333_W + tank.pacman.pos.x] == 63))

					movTank_x = -1;
			}
			else
				movTank_x = 0;
			tank.pacman.dir = LEFT;
		}
		else if(moveTank == 'w')
		{
			if(tank.pacman.pos.y >= 1)
			{
				if(((unpack_rgb333_p32[(tank.pacman.pos.y-1)*SCREEN_RGB333_W + tank.pacman.pos.x + 2] == 0) &&
				 (unpack_rgb333_p32[(tank.pacman.pos.y-1)*SCREEN_RGB333_W + 13 + tank.pacman.pos.x] == 0)) ||
					(unpack_rgb333_p32[(tank.pacman.pos.y-1)*SCREEN_RGB333_W + tank.pacman.pos.x + 2] == 63) ||
					(unpack_rgb333_p32[(tank.pacman.pos.y-1)*SCREEN_RGB333_W + 13 + tank.pacman.pos.x] == 63))

					movTank_y = -1;
			}
			else
				movTank_y = 0;
			tank.pacman.dir = UP;
		}
		else if(moveTank == 's')
		{
			if(tank.pacman.pos.y <= 208 - 16)
			{
				if(((unpack_rgb333_p32[(tank.pacman.pos.y+17)*SCREEN_RGB333_W + tank.pacman.pos.x + 2] == 0) &&
				 (unpack_rgb333_p32[(tank.pacman.pos.y+17)*SCREEN_RGB333_W + 13 + tank.pacman.pos.x] == 0)) ||
				(unpack_rgb333_p32[(tank.pacman.pos.y+17)*SCREEN_RGB333_W + tank.pacman.pos.x + 2] == 63) ||
				(unpack_rgb333_p32[(tank.pacman.pos.y+17)*SCREEN_RGB333_W + 13 + tank.pacman.pos.x] == 63))

					movTank_y = +1;
			}
			else
				movTank_y = 0;
			tank.pacman.dir = DOWN;
		}
		else if(moveTank == 'q')
		{
				bul = true;
				bullet.pacman.dir = tank.pacman.dir;
				switch(tank.pacman.dir)
				{
					case UP:
						if (tank.pacman.pos.y > 2)
						{
							bullet.pacman.pos.x = tank.pacman.pos.x ;
							bullet.pacman.pos.y = tank.pacman.pos.y - 7;
							bullet.pacman.dir = UP;
						}
						break;
					case DOWN:
						bullet.pacman.pos.x = tank.pacman.pos.x + 0;
						bullet.pacman.pos.y = tank.pacman.pos.y + 10;
						bullet.pacman.dir = DOWN;
						break;
					case LEFT:
						bullet.pacman.pos.x = tank.pacman.pos.x - 9;
						bullet.pacman.pos.y = tank.pacman.pos.y;
						bullet.pacman.dir = LEFT;
						break;
					case RIGHT:
						bullet.pacman.pos.x = tank.pacman.pos.x + 10;
						bullet.pacman.pos.y = tank.pacman.pos.y + 1;
						bullet.pacman.dir = RIGHT;
						break;
				}
		}
		
		if(bul)
		{
			while(skip < 2)
			{
				skip++;
				switch(bullet.pacman.dir)
				{
					case UP:
						if(bullet.pacman.pos.y > 1)
						{
							if(((unpack_rgb333_p32[(bullet.pacman.pos.y-1)*SCREEN_RGB333_W + bullet.pacman.pos.x + 2] == 0) && 
							(unpack_rgb333_p32[(bullet.pacman.pos.y-1)*SCREEN_RGB333_W + 13 + bullet.pacman.pos.x] == 0))||
							(unpack_rgb333_p32[(bullet.pacman.pos.y-1)*SCREEN_RGB333_W + bullet.pacman.pos.x + 2] == 63) ||
							(unpack_rgb333_p32[(bullet.pacman.pos.y-1)*SCREEN_RGB333_W + 13 + bullet.pacman.pos.x] == 63))
								bullet.pacman.pos.y--;
							else
								bul = false;
						}
						else
							bul = false;
						break;
					case DOWN:
						if(bullet.pacman.pos.y < 208)
						{
							if(((unpack_rgb333_p32[(bullet.pacman.pos.y+17)*SCREEN_RGB333_W + bullet.pacman.pos.x + 2] == 0) &&
 							(unpack_rgb333_p32[(bullet.pacman.pos.y+17)*SCREEN_RGB333_W + 13 + bullet.pacman.pos.x] == 0))||
							(unpack_rgb333_p32[(bullet.pacman.pos.y+17)*SCREEN_RGB333_W + bullet.pacman.pos.x + 2] == 63) ||
							(unpack_rgb333_p32[(bullet.pacman.pos.y+17)*SCREEN_RGB333_W + 13 + bullet.pacman.pos.x] == 63))
								bullet.pacman.pos.y++;
							else
								bul = false;
						}
						break;
					case LEFT:
						if(bullet.pacman.pos.x > 1)
						{
							if(((unpack_rgb333_p32[(bullet.pacman.pos.y+2)*SCREEN_RGB333_W + bullet.pacman.pos.x] == 0) &&
 							(unpack_rgb333_p32[(bullet.pacman.pos.y+13)*SCREEN_RGB333_W + bullet.pacman.pos.x] == 0))||
							 (unpack_rgb333_p32[(bullet.pacman.pos.y+2)*SCREEN_RGB333_W + bullet.pacman.pos.x] == 63) ||
				 			(unpack_rgb333_p32[(bullet.pacman.pos.y+13)*SCREEN_RGB333_W + bullet.pacman.pos.x] == 63))
								bullet.pacman.pos.x--;
							else
							{
								bul = false;
							}
						}
						else
							bul = false;
						break;
					case RIGHT:
						if(bullet.pacman.pos.x < 208)
						{
							if(((unpack_rgb333_p32[(bullet.pacman.pos.y+2)*SCREEN_RGB333_W + 15 + bullet.pacman.pos.x] == 0) &&
				 			(unpack_rgb333_p32[(bullet.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + bullet.pacman.pos.x] == 0))||
							(unpack_rgb333_p32[(bullet.pacman.pos.y+2)*SCREEN_RGB333_W + 15 + bullet.pacman.pos.x] == 63) ||
							(unpack_rgb333_p32[(bullet.pacman.pos.y+13)*SCREEN_RGB333_W + 15 + bullet.pacman.pos.x] == 63))
								bullet.pacman.pos.x++;
							else 
								bul = false;
						}
						break;
				}
				
				draw_bullet_from_atlas(0, 0, 16, 16, bullet.pacman.pos.x, bullet.pacman.pos.y);
				if(!bul)
				{
					bullet.pacman.pos.x = 300;
					bullet.pacman.pos.y = 300;
				}

			}
		}
		skip = 0;
		
		if(cnt == 16)
		{
			movement = ' ';
			moveTank = ' ';
			cnt = 0;
		}

			gs.pacman.pos.x += mov_x*STEP;
			gs.pacman.pos.y += mov_y*STEP;

			tank.pacman.pos.x += movTank_x*STEP;
			tank.pacman.pos.y += movTank_y*STEP;

		usleep(10);

		switch(gs.pacman.anim.state){
		case PACMAN_IDLE:
			if(mov_x != 0 || mov_y != 0)
			{
				gs.pacman.anim.delay_cnt = PACMAN_ANIM_DELAY;
				gs.pacman.anim.state = PACMAN_WITH_OPEN_MOUTH;
			}
			break;
		case PACMAN_OPENING_MOUTH:
			if(gs.pacman.anim.delay_cnt != 0)
			{
					gs.pacman.anim.delay_cnt--;
			}
			else
			{
				gs.pacman.anim.delay_cnt = PACMAN_ANIM_DELAY;
				gs.pacman.anim.state = PACMAN_WITH_OPEN_MOUTH;
			}
			break;
		case PACMAN_WITH_OPEN_MOUTH:
			if(gs.pacman.anim.delay_cnt != 0)
			{
					gs.pacman.anim.delay_cnt--;
			}
			else
			{
				if(mov_x != 0 || mov_y != 0)
				{
					gs.pacman.anim.delay_cnt = PACMAN_ANIM_DELAY;
					gs.pacman.anim.state = PACMAN_CLOSING_MOUTH;
				}
				else
				{
					gs.pacman.anim.state = PACMAN_IDLE;
				}
			}
			break;
		case PACMAN_CLOSING_MOUTH:
			if(gs.pacman.anim.delay_cnt != 0)
			{
					gs.pacman.anim.delay_cnt--;
			}else
			{
				gs.pacman.anim.delay_cnt = PACMAN_ANIM_DELAY;
				gs.pacman.anim.state = PACMAN_WITH_CLOSED_MOUTH;
			}
			break;
		case PACMAN_WITH_CLOSED_MOUTH:
			if(gs.pacman.anim.delay_cnt != 0)
			{
					gs.pacman.anim.delay_cnt--;
			}
			else
			{
				if(mov_x != 0 || mov_y != 0)
				{
					gs.pacman.anim.delay_cnt = PACMAN_ANIM_DELAY;
					gs.pacman.anim.state = PACMAN_OPENING_MOUTH;
				}
				else
				{
					gs.pacman.anim.state = PACMAN_IDLE;
				}
			}
			break;
		}

		WAIT_UNITL_0(gpu_p32[2]);
		WAIT_UNITL_1(gpu_p32[2]);
		
		draw_map_from_atlas(0, 0, 208, 208, 0, 0);

		if((gs.pacman.pos.x - bullet.pacman.pos.x > - 7) && (gs.pacman.pos.x - bullet.pacman.pos.x < 7) && (gs.pacman.pos.y - bullet.pacman.pos.y > - 7) && (gs.pacman.pos.y - bullet.pacman.pos.y < 7))
		{
			tankWon = true;
			break;
		}
		if(sqrt(((gs.pacman.pos.x+8) - (tank.pacman.pos.x+8))*((gs.pacman.pos.x+8) - (tank.pacman.pos.x+8)) + ((gs.pacman.pos.y+8) - (tank.pacman.pos.y+8))*((gs.pacman.pos.y+8 - tank.pacman.pos.y+8))) < 8)
		{
			pacWon = true;
			break;
		}
		
		switch(gs.pacman.anim.state)
		{
			case PACMAN_IDLE:
			case PACMAN_OPENING_MOUTH:
			case PACMAN_CLOSING_MOUTH:
				// Half open mouth.

				if(gs.pacman.dir == RIGHT)
					draw_sprite_from_atlas(16, 0, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				else if(gs.pacman.dir == LEFT)
					draw_sprite_from_atlas(16, 16, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				else if(gs.pacman.dir == UP)
					draw_sprite_from_atlas(16, 32, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				else if(gs.pacman.dir == DOWN)
					draw_sprite_from_atlas(16, 48, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				break;
			case PACMAN_WITH_OPEN_MOUTH:
				// Full open mouth.
				if(gs.pacman.dir == RIGHT)
					draw_sprite_from_atlas(0, 0, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				else if(gs.pacman.dir == LEFT)
					draw_sprite_from_atlas(0, 16, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				else if(gs.pacman.dir == UP)
					draw_sprite_from_atlas(0, 32, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				else if(gs.pacman.dir == DOWN)
					draw_sprite_from_atlas(0, 48, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				break;
			case PACMAN_WITH_CLOSED_MOUTH:
				// Close mouth.
				draw_sprite_from_atlas(32, 0, 16, 16, gs.pacman.pos.x, gs.pacman.pos.y);
				break;
		}

		switch(tank.pacman.dir)
		{
			case UP:
				draw_tank_from_atlas(128, 0, 16, 16, tank.pacman.pos.x, tank.pacman.pos.y);
				break;
			case LEFT:
				draw_tank_from_atlas(160, 0, 16, 16, tank.pacman.pos.x, tank.pacman.pos.y);
				break;
			case DOWN:
					draw_tank_from_atlas(192, 0, 16, 16, tank.pacman.pos.x, tank.pacman.pos.y);
					break;
			case RIGHT:
					draw_tank_from_atlas(224, 0, 16, 16, tank.pacman.pos.x, tank.pacman.pos.y);
					break;
		}
	}

	prev_direction = gs.pacman.dir;
	
	if(pacWon)
		printf("Pacman wins\n");
	if(tankWon)
		printf("Tank wins\n");

	printf("Game over\n");

	return 0;
}

///////////////////////////////////////////////////////////////////////////////




