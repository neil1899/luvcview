/*******************************************************************************
#	 	luvcview: Sdl video Usb Video Class grabber           .        #
#This package work with the Logitech UVC based webcams with the mjpeg feature. #
#All the decoding is in user space with the embedded jpeg decoder              #
#.                                                                             #
# 		Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard     #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <string.h>
#include <pthread.h>
#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>
#include <SDL/SDL_audio.h>
#include <SDL/SDL_timer.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <X11/Xlib.h>
#include <SDL/SDL_syswm.h>
#include "v4l2uvc.h"
#include "gui.h"
#include "utils.h"
#include "color.h"

/* Fixed point arithmetic */
#define FIXED Sint32
#define FIXED_BITS 16
#define TO_FIXED(X) (((Sint32)(X))<<(FIXED_BITS))
#define FROM_FIXED(X) (((Sint32)(X))>>(FIXED_BITS))

#define INCPANTILT 64 // 1°

typedef enum action_gui {
/* 0..7..15 action top */
    A_BRIGHTNESS_UP,
    A_CONTRAST_UP,
    A_SATURATION_UP,
    A_GAIN_UP,
    A_SHARPNESS_UP,
    A_GAMMA_UP,
    A_SCREENSHOT,
    A_RESET,
    A_PAN_UP,
    A_TILT_UP,
    A_PAN_RESET,
    A_SWITCH_LIGHTFREQFILT,
    A_EXPOSURE_UP,
    A_EXPOSURE_ON,
    A_BALANCE_UP,
    A_BALANCE_ON,
/* 8..15 -> 16..31 action bottom */
    A_BRIGHTNESS_DOWN,
    A_CONTRAST_DOWN,
    A_SATURATION_DOWN,
    A_GAIN_DOWN,
    A_SHARPNESS_DOWN,
    A_GAMMA_DOWN,
    A_RECORD_TOGGLE,
    A_QUIT,
    A_PAN_DOWN,
    A_TILT_DOWN,
    A_TILT_RESET,
    A_NOT1_DOWN,
    A_EXPOSURE_DOWN,
    A_EXPOSURE_OFF,
    A_BALANCE_DOWN,
    A_BALANCE_OFF,
/* 16.. action others */
    A_VIDEO,
    A_DEBUG,
    A_CAPTURE_FRAME,
    A_CAPTURE_STREAM,
    A_CAPTURE_FRAMESTREAM,
    A_SAVE,
    A_LOAD,
    A_LAST
} action_gui;

typedef struct act_title {
	action_gui action;
	char * title;
} act_title;

typedef struct key_action_t {
    SDLKey key;
    action_gui action;
} key_action_t;

key_action_t keyaction[] = {
    {SDLK_n, A_BRIGHTNESS_UP},
    {SDLK_b, A_BRIGHTNESS_DOWN},
    {SDLK_x, A_CONTRAST_UP},
    {SDLK_w, A_CONTRAST_DOWN},
    {SDLK_c, A_SATURATION_UP},
    {SDLK_v, A_SATURATION_DOWN},
    {SDLK_z, A_GAIN_UP},
    {SDLK_a, A_GAIN_DOWN},
    {SDLK_r, A_SHARPNESS_UP},
    {SDLK_e, A_SHARPNESS_DOWN},
    {SDLK_y, A_PAN_UP},
    {SDLK_t, A_PAN_DOWN},
    {SDLK_s, A_SCREENSHOT},
    {SDLK_p, A_RECORD_TOGGLE},
    {SDLK_f, A_SWITCH_LIGHTFREQFILT},
    {SDLK_l, A_RESET},
    {SDLK_q, A_QUIT},
    {SDLK_m, A_VIDEO},
    {SDLK_d, A_DEBUG},
    {SDLK_f, A_CAPTURE_FRAME},
    {SDLK_i, A_CAPTURE_STREAM},
    {SDLK_j, A_CAPTURE_FRAMESTREAM},
    {SDLK_F1, A_SAVE},
    {SDLK_F2, A_LOAD}
};

act_title title_act[A_LAST] = {
/* 0..7..15 action top */
   { A_BRIGHTNESS_UP,"Brightness Up"},
   { A_CONTRAST_UP,"Contrast Up"},
   { A_SATURATION_UP,"Saturation Up"},
   { A_GAIN_UP,"Gain_Up"},
   { A_SHARPNESS_UP,"Sharpness Up"},
   { A_GAMMA_UP,"Gamma Up"},
   { A_SCREENSHOT,"Take a Picture!!"},
   { A_RESET,"Reset All to Default !!"},
   { A_PAN_UP,"Pan +angle"},
   { A_TILT_UP,"Tilt +angle"},
   { A_PAN_RESET,"Pan reset"},
   { A_SWITCH_LIGHTFREQFILT,"Switch light freq filter"},
   { A_EXPOSURE_UP,"Exposure Up"},
   { A_EXPOSURE_ON,"Auto Exposure On"},
   { A_BALANCE_UP,"White Balance Up"},
   { A_BALANCE_ON,"Auto White Balance On"},
/* 8..15 -> 16..31 action bottom */
   { A_BRIGHTNESS_DOWN,"Brightness Down"},
   { A_CONTRAST_DOWN,"Contrast Down"},
   { A_SATURATION_DOWN,"Saturation Down"},
   { A_GAIN_DOWN,"Gain Down"},
   { A_SHARPNESS_DOWN,"Sharpness Down"},
   { A_GAMMA_DOWN,"Gamma Down"},
   { A_RECORD_TOGGLE,"AVI Start/Stop"},
   { A_QUIT,"Quit Happy, Bye Bye:)"},
   { A_PAN_DOWN,"Pan -angle"},
   { A_TILT_DOWN,"Tilt -angle"},
   { A_TILT_RESET,"Tilt Reset"},
   { A_NOT1_DOWN,"Nothing"},
   { A_EXPOSURE_DOWN,"Exposure Down"},
   { A_EXPOSURE_OFF,"Auto Exposure OFF"},
   { A_BALANCE_DOWN,"White Balance Down"},
   { A_BALANCE_OFF,"Auto White Balance OFF"},
/* 16.. action others */
   { A_VIDEO,"LUVCview"},
   { A_CAPTURE_FRAME, "Single frame captured" },
   { A_CAPTURE_STREAM, "Stream capture" },
   { A_CAPTURE_FRAMESTREAM, "Frame stream capture" },
   { A_SAVE, "Saved Configuration" },
   { A_LOAD, "Restored Configuration" }
};
static const char version[] = VERSION;

struct vdIn *videoIn_left;
struct vdIn *videoIn_right;

static uint64_t left_time_start = -1;
static uint64_t left_time_end = -1;
static uint64_t right_time_start = -1;
static uint64_t right_time_end = -1;
static uint64_t trackre_time_start = -1;
static uint64_t trackre_time_end = -1;

static int total_frames_left = 0;
static int total_frames_right = 0;
static int total_frames_trackre = 0;

extern uint64_t jpeg_time_total;

extern int post_screen_flag;

static uint64_t vcos_getmicrosecs64_internal(void)
{
   struct timeval tv;
   uint64_t tm = 0;

   if (!gettimeofday(&tv, NULL))
   {
      tm = (tv.tv_sec * 1000000LL) + tv.tv_usec;
   }

   return tm;
}

/* Translates screen coordinates into buttons */
action_gui
GUI_whichbutton(int x, int y, SDL_Surface * pscreen, struct vdIn *videoIn);

action_gui GUI_keytoaction(SDLKey key);

struct pt_data {
    SDL_Surface **ptscreen;
    SDL_Event *ptsdlevent;
    SDL_Rect *drect_left;
	SDL_Rect *drect_right;
    struct vdIn *ptvideoIn_left;
	struct vdIn *ptvideoIn_right;
    float frmrate;
    SDL_mutex *affmutex;
} pt_common_data;

struct pt_camera_data {
    SDL_Overlay **ptoverlay;
    SDL_Rect *drect;
    struct vdIn *ptvideoIn;
    float frmrate;
    SDL_mutex *affmutex;
	SDL_mutex *trackretex;
} pt_camera_left_data, pt_camera_right_data;

struct pt_trackre_data {
    struct vdIn *ptvideoIn_left;
	struct vdIn *ptvideoIn_right;
	SDL_mutex *trackretex_left;
	SDL_mutex *trackretex_right;
} pt_common_trackre_data;

static int eventThread(void *data);
static int eventthread_camera_left(void *data);
static int eventthread_camera_right(void *data);
static int eventthread_trackre(void *data);

static Uint32 SDL_VIDEO_Flags =
    SDL_ANYFORMAT | SDL_DOUBLEBUF | SDL_RESIZABLE;

static void signal_handler(int signal_number)
{
   left_time_end = vcos_getmicrosecs64_internal()/1000;
   right_time_end = vcos_getmicrosecs64_internal()/1000;
   trackre_time_end = vcos_getmicrosecs64_internal()/1000;

   printf("video0 total frames %d, spend time is %lldms\n", total_frames_left, (left_time_end - left_time_start));
   printf("video1 total frames %d, spend time is %lldms\n", total_frames_right, (right_time_end - right_time_start));
   printf("trackre total frames %d, spend time is %lldms\n", total_frames_trackre, (trackre_time_end - trackre_time_start));
   printf("video0 jpeg total frames %d, spend time is %lldms-----%llds\n", total_frames_left, jpeg_time_total, jpeg_time_total_1);
}

int main(int argc, char *argv[])
{
	const SDL_VideoInfo *info;
	char driver[128];
	SDL_Surface *pscreen;

	SDL_Overlay *overlay_left;
	SDL_Overlay *overlay_right;
	SDL_Rect drect_left;
	SDL_Rect drect_right;

	SDL_Event sdlevent;

	SDL_Thread *mythread, *camera_left_thread, *camera_right_thread, *camera_trackre;

	SDL_mutex *affmutex, *trackre_left, *trackre_right;

	int i;
	int status, status_left, status_right, status_trackre;

	unsigned char *p_left = NULL;
	unsigned char *p_right = NULL;
	const char *videodevice_left = "/dev/video0";
	const char *videodevice_right = "/dev/video1";

	int width = 640;
	int height = 360;
	float fps = 60.0;			// Requested frame rate
	int format = V4L2_PIX_FMT_MJPEG;

	int hwaccel = 0;
	int grabmethod = 1;
	const char *mode = NULL;
	char *sizestring = NULL;
	char *separateur = NULL;
	int queryformats = 0;
	int querycontrols = 0;
	int readconfigfile = 0;
	int enableRawStreamCapture = 0;
	int enableRawFrameCapture = 0;
	char *fpsstring  = NULL;
	float frmrate = 0.0;		// Measured frame rate

	char *avifilename_left = NULL;
	char *avifilename_right = NULL;

	printf("luvcview %s\n\n", version);

	signal(SIGINT, signal_handler);

	for (i = 1; i < argc; i++)
	{
		/* skip bad arguments */
		if (argv[i] == NULL || *argv[i] == 0 || *argv[i] != '-') {
			continue;
		}

		if (strcmp(argv[i], "-g") == 0) {
			/* Ask for read instead default  mmap */
			grabmethod = 0;
		}
		if (strcmp(argv[i], "-w") == 0) {
			/* disable hw acceleration */
			hwaccel = 1;
		}
		if (strcmp(argv[i], "-f") == 0) {
			if (i + 1 >= argc) {
				printf("No parameter specified with -f, aborting.\n");
				exit(1);
			}
			mode = argv[i + 1];

			if (strcasecmp(mode, "yuv") == 0 || strcasecmp(mode, "YUYV") == 0) {
				format = V4L2_PIX_FMT_YUYV;
			} else if (strcasecmp(mode, "jpg") == 0 || strcasecmp(mode, "MJPG") == 0) {
				format = V4L2_PIX_FMT_MJPEG;
			} else {
				printf("Unknown format specified. Aborting.\n");
				exit(1);
			}
		}

		if (strcmp(argv[i], "-s") == 0) {
			if (i + 1 >= argc) {
				printf("No parameter specified with -s, aborting.\n");
				exit(1);
			}

			sizestring = strdup(argv[i + 1]);

			width = strtoul(sizestring, &separateur, 10);
			if (*separateur != 'x') {
				printf("Error in size use -s widthxheight\n");
				exit(1);
			} else {
				++separateur;
				height = strtoul(separateur, &separateur, 10);
				if (*separateur != 0)
					printf("hmm.. dont like that!! trying this height\n");
			}
		}

		if (strcmp(argv[i], "-i") == 0){
			if (i + 1 >= argc) {
				printf("No parameter specified with -i, aborting.\n");
				exit(1);
			}
			fpsstring = argv[i + 1];
			fps = strtof(fpsstring, &separateur);
			if(*separateur != '\0') {
				printf("Invalid frame rate '%s' specified with -i. "
						"Only numbers are supported. Aborting.\n", fpsstring);
				exit(1);
			}
		}
		if (strcmp(argv[i], "-S") == 0) {
			/* Enable raw stream capture from the start */
			enableRawStreamCapture = 1;
		}
		if (strcmp(argv[i], "-c") == 0) {
			/* Enable raw frame capture for the first frame */
			enableRawFrameCapture = 1;
		}
		if (strcmp(argv[i], "-C") == 0) {
			/* Enable raw frame stream capture from the start*/
			enableRawFrameCapture = 2;
		}

		if (strcmp(argv[i], "-L") == 0) {
			/* query list of valid video formats */
			queryformats = 1;
		}
		if (strcmp(argv[i], "-l") == 0) {
			/* query list of valid video formats */
			querycontrols = 1;
		}

		if (strcmp(argv[i], "-r") == 0) {
			/* query list of valid video formats */
			readconfigfile = 1;
		}

		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("usage: uvcview [-h -d -g -f -s -i -c -o -C -S -L -l -r]\n");
			printf("-h	 print this message\n");
			printf("-d	 /dev/videoX	   use videoX device\n");
			printf("-g	 use read method for grab instead mmap\n");
			printf("-w	 disable SDL hardware accel.\n");
			printf("-f	 choose video format (YUYV/yuv and MJPG/jpg are valid, MJPG is default)\n");
			printf("-i	 fps		   use specified frame rate\n");
			printf("-s	 widthxheight	   use specified input size\n");
			printf("-c	 enable raw frame capturing for the first frame\n");
			printf("-C	 enable raw frame stream capturing from the start\n");
			printf("-S	 enable raw stream capturing from the start\n");
			printf("-o	 avifile  create avifile, default video.avi\n");
			printf("-L	 query valid video formats\n");
			printf("-l	 query valid controls and settings\n");
			printf("-r	 read and set control settings from luvcview.cfg (save/restore with F1/F2)\n");
			exit(0);
		}
	}

	/************* Test SDL capabilities ************/
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
		exit(1);
	}

	/* For this version, we'll be save and disable hardware acceleration */
	if(hwaccel)
		if ( ! getenv("SDL_VIDEO_YUV_HWACCEL") ) {
			putenv("SDL_VIDEO_YUV_HWACCEL=0");
		}

	printf("SDL information:\n");

	if (SDL_VideoDriverName(driver, sizeof(driver))) {
		printf("  Video driver: %s\n", driver);
	}
	info = SDL_GetVideoInfo();

	if (info->wm_available) {
		printf("  A window manager is available\n");
	}
	if (info->hw_available) {
		printf("  Hardware surfaces are available (%dk video memory)\n",
				info->video_mem);
		SDL_VIDEO_Flags |= SDL_HWSURFACE;
	}
	if (info->blit_hw) {
		printf("  Copy blits between hardware surfaces are accelerated\n");
		SDL_VIDEO_Flags |= SDL_ASYNCBLIT;
	}
	if (info->blit_hw_CC) {
		printf
			("  Colorkey blits between hardware surfaces are accelerated\n");
	}
	if (info->blit_hw_A) {
		printf("  Alpha blits between hardware surfaces are accelerated\n");
	}
	if (info->blit_sw) {
		printf
			("  Copy blits from software surfaces to hardware surfaces are accelerated\n");
	}
	if (info->blit_sw_CC) {
		printf
			("  Colorkey blits from software surfaces to hardware surfaces are accelerated\n");
	}
	if (info->blit_sw_A) {
		printf
			("  Alpha blits from software surfaces to hardware surfaces are accelerated\n");
	}
	if (info->blit_fill) {
		printf("  Color fills on hardware surfaces are accelerated\n");
	}

	if (!(SDL_VIDEO_Flags & SDL_HWSURFACE))
		SDL_VIDEO_Flags |= SDL_SWSURFACE;

	if (avifilename_left == NULL || *avifilename_left == 0) {
		avifilename_left = "video_left.avi";
	}

	if (avifilename_right == NULL || *avifilename_right == 0) {
		avifilename_right = "video_right.avi";
	}

	videoIn_left = (struct vdIn *) calloc(1, sizeof(struct vdIn));
	if ( queryformats ) {
		/* if we're supposed to list the video formats, do that now and go out */
		check_videoIn(videoIn_left, (char *)videodevice_left);
		free(videoIn_left);
		SDL_Quit();
		exit(1);
	}
	if (init_videoIn
			(videoIn_left, (char *)videodevice_left, width, height, fps, format,
			 grabmethod, avifilename_left) < 0)
		exit(1);
	/* if we're supposed to list the controls, do that now */
	if ( querycontrols )
		enum_controls(videoIn_left->fd);

	/* if we're supposed to read the control settings from a configfile, do that now */
	if ( readconfigfile )
		load_controls(videoIn_left->fd);

	videoIn_right = (struct vdIn *) calloc(1, sizeof(struct vdIn));
	if ( queryformats ) {
		/* if we're supposed to list the video formats, do that now and go out */
		check_videoIn(videoIn_right, (char *)videodevice_right);
		free(videoIn_right);
		SDL_Quit();
		exit(1);
	}
	if (init_videoIn
			(videoIn_right, (char *) videodevice_right, width, height, fps, format,
			 grabmethod, avifilename_right) < 0)
		exit(1);
	/* if we're supposed to list the controls, do that now */
	if ( querycontrols )
		enum_controls(videoIn_right->fd);

	/* if we're supposed to read the control settings from a configfile, do that now */
	if ( readconfigfile )
		load_controls(videoIn_right->fd);

	pscreen =
		SDL_SetVideoMode((videoIn_left->width+videoIn_right->width+32), videoIn_left->height + 32, 0,
				SDL_VIDEO_Flags);

	overlay_left =
		SDL_CreateYUVOverlay(videoIn_left->width, videoIn_left->height + 32,
				SDL_YUY2_OVERLAY, pscreen);
	p_left = (unsigned char *)overlay_left->pixels[0];
	drect_left.x = 0;
	drect_left.y = 0;
	drect_left.w = videoIn_left->width;
	drect_left.h = videoIn_left->height + 32;

	overlay_right =
		SDL_CreateYUVOverlay(videoIn_right->width, videoIn_right->height + 32,
				SDL_YUY2_OVERLAY, pscreen);
	p_right = (unsigned char *)overlay_right->pixels[0];
	drect_right.x = videoIn_right->width+32;
	drect_right.y = 0;
	drect_right.w = videoIn_right->width;
	drect_right.h = videoIn_right->height + 32;

	if (enableRawStreamCapture) {
		videoIn_left->captureFile = fopen("stream_left.raw", "wb");
		if(videoIn_left->captureFile == NULL) {
			perror("Unable to open file for raw stream capturing");
		} else {
			printf("Starting raw stream capturing to stream.raw ...\n");
		}

		videoIn_right->captureFile = fopen("stream_right.raw", "wb");
		if(videoIn_right->captureFile == NULL) {
			perror("Unable to open file for raw stream capturing");
		} else {
			printf("Starting raw stream capturing to stream.raw ...\n");
		}
	}

	if (enableRawFrameCapture) {
		videoIn_left->rawFrameCapture = enableRawFrameCapture;
		videoIn_right->rawFrameCapture = enableRawFrameCapture;
	}

	initLut_Left();
	initLut_Right();

	SDL_WM_SetCaption(title_act[A_VIDEO].title, NULL);

	creatButt(videoIn_left->width, 32);

	SDL_LockYUVOverlay(overlay_left);
	memcpy(p_left + (videoIn_left->width * (videoIn_left->height) * 2), YUYVbutt,
			videoIn_left->width * 32 * 2);
	SDL_UnlockYUVOverlay(overlay_left);

	SDL_LockYUVOverlay(overlay_right);
	memcpy(p_right + (videoIn_right->width * (videoIn_right->height) * 2), YUYVbutt,
			videoIn_right->width * 32 * 2);
	SDL_UnlockYUVOverlay(overlay_right);

	/* initialize thread data */
	pt_common_data.ptscreen = &pscreen;
	pt_common_data.ptvideoIn_left = videoIn_left;
	pt_common_data.ptvideoIn_right = videoIn_right;
	pt_common_data.ptsdlevent = &sdlevent;
	pt_common_data.drect_left = &drect_left;
	pt_common_data.drect_right = &drect_right;
	affmutex = SDL_CreateMutex();
	pt_common_data.affmutex = affmutex;

	trackre_left = SDL_CreateMutex();
	trackre_right = SDL_CreateMutex();

	pt_camera_left_data.ptoverlay = &overlay_left;
	pt_camera_left_data.ptvideoIn = videoIn_left;
	pt_camera_left_data.drect = &drect_left;
	pt_camera_left_data.affmutex = affmutex;
	pt_camera_left_data.trackretex = trackre_left;

	pt_camera_right_data.ptoverlay = &overlay_right;
	pt_camera_right_data.ptvideoIn = videoIn_right;
	pt_camera_right_data.drect = &drect_right;
	pt_camera_right_data.affmutex = affmutex;
	pt_camera_right_data.trackretex = trackre_right;

	pt_common_trackre_data.ptvideoIn_left = videoIn_left;
	pt_common_trackre_data.ptvideoIn_right = videoIn_right;
	pt_common_trackre_data.trackretex_left = trackre_left;
	pt_common_trackre_data.trackretex_right = trackre_right;

	mythread = SDL_CreateThread(eventThread, (void *) &pt_common_data);
	camera_left_thread = SDL_CreateThread(eventthread_camera_left, (void *) &pt_camera_left_data);
	camera_right_thread = SDL_CreateThread(eventthread_camera_right, (void *) &pt_camera_right_data);
	camera_trackre = SDL_CreateThread(eventthread_trackre, (void *) &pt_common_trackre_data);

	SDL_WaitThread(mythread, &status);
	SDL_WaitThread(camera_left_thread, &status_left);
	SDL_WaitThread(camera_right_thread, &status_right);
	SDL_WaitThread(camera_trackre, &status_trackre);

	SDL_DestroyMutex(affmutex);
	SDL_DestroyMutex(trackre_left);
	SDL_DestroyMutex(trackre_right);

	/* if avifile is defined, we made a video: compute the exact fps and
	   set it in the video */
	if (videoIn_left->avifile != NULL) {
		float fps=(videoIn_left->framecount/(videoIn_left->recordtime/1000));
		fprintf(stderr,"setting fps to %f\n",fps);
		AVI_set_video(videoIn_left->avifile, videoIn_left->width, videoIn_left->height,
				fps, "MJPG");
		AVI_close(videoIn_left->avifile);
	}

	if (videoIn_right->avifile != NULL) {
		float fps=(videoIn_right->framecount/(videoIn_right->recordtime/1000));
		fprintf(stderr,"setting fps to %f\n",fps);
		AVI_set_video(videoIn_right->avifile, videoIn_right->width, videoIn_right->height,
				fps, "MJPG");
		AVI_close(videoIn_right->avifile);
	}

	close_v4l2(videoIn_left);
	free(videoIn_left);
	close_v4l2(videoIn_right);
	free(videoIn_right);
	destroyButt();
	freeLut_Left();
	freeLut_Right();
	printf("Cleanup done. Exiting ...\n");
	SDL_Quit();
}

action_gui
GUI_whichbutton(int x, int y, SDL_Surface * pscreen, struct vdIn *videoIn)
{
    int nbutton, retval;
    FIXED scaleh = TO_FIXED(pscreen->h) / (videoIn->height + 32);
    int nheight = FROM_FIXED(scaleh * videoIn->height);
    if (y < nheight)
	return (A_VIDEO);
    nbutton = FROM_FIXED(scaleh * 32);
    /* 8 buttons across the screen, corresponding to 0-7 extand to 16*/
    retval = (x * 16) / (videoIn->width);
    /* Bottom half of the button denoted by flag|0x10 */
    if (y > (nheight + (nbutton / 2)))
	retval |= 0x10;
    return ((action_gui) retval);
}

action_gui GUI_keytoaction(SDLKey key)
{
	int i = 0;
	while(keyaction[i].key){
		if (keyaction[i].key == key)
			return (keyaction[i].action);
		i++;
	}

	return (A_VIDEO);
}

static int eventThread(void *data)
{
	struct pt_data *gdata = (struct pt_data *)data;
	struct v4l2_control control;
	SDL_Surface *pscreen = *gdata->ptscreen;
	struct vdIn *videoIn_left = gdata->ptvideoIn_left;
	struct vdIn *videoIn_right = gdata->ptvideoIn_right;
	SDL_Event *sdlevent = gdata->ptsdlevent;
	SDL_Rect *drect_left = gdata->drect_left;
	SDL_Rect *drect_right = gdata->drect_right;
	SDL_mutex *affmutex = gdata->affmutex;
	int x, y;
	int mouseon = 0;
	int value = 0;
	int len = 0;
	short incpantilt = INCPANTILT;
	int boucle = 0;
	action_gui curr_action = A_VIDEO;
	while (videoIn_left->signalquit) {
		SDL_LockMutex(affmutex);
		float frmrate = gdata->frmrate;
		while (SDL_PollEvent(sdlevent)) {	//scan the event queue
			switch (sdlevent->type) {
				case SDL_KEYUP:
				case SDL_MOUSEBUTTONUP:
					mouseon = 0;
					incpantilt = INCPANTILT;
					boucle = 0;
					break;
				case SDL_MOUSEBUTTONDOWN:
					mouseon = 1;
				case SDL_MOUSEMOTION:
					SDL_GetMouseState(&x, &y);
					curr_action = GUI_whichbutton(x, y, pscreen, videoIn_left);
					break;
				/*case SDL_VIDEORESIZE:
					pscreen =
						SDL_SetVideoMode(sdlevent->resize.w,
								sdlevent->resize.h, 0,
								SDL_VIDEO_Flags);
					drect_left->x = 0;
					drect_left->y = 0;
					drect_left->w = sdlevent->resize.w;
					drect_left->h = sdlevent->resize.h;

					drect_right->w = sdlevent->resize.w;
					drect_right->h = sdlevent->resize.h;
					drect_right->x = drect_right->w+32;
					drect_right->y = 0;
					printf("resize w is %d, resize y is %d\n", sdlevent->resize.w, sdlevent->resize.h);
					break;*/
				case SDL_KEYDOWN:
					curr_action = GUI_keytoaction(sdlevent->key.keysym.sym);
					if (curr_action != A_VIDEO)
						mouseon = 1;
					break;
				case SDL_QUIT:
					printf("\nQuit signal received.\n");
					videoIn_left->signalquit = 0;
					videoIn_right->signalquit = 0;
					break;
			}
		}			//end if poll
		SDL_UnlockMutex(affmutex);
		/* traiter les actions */
		value = 0;
		if (mouseon){
			boucle++;
			switch (curr_action) {
				case A_BRIGHTNESS_UP:
					if ((value = v4l2UpControl(videoIn_left, V4L2_CID_BRIGHTNESS)) < 0)
						printf("Set LEFT Brightness up error\n");
					if ((value = v4l2UpControl(videoIn_right, V4L2_CID_BRIGHTNESS)) < 0)
						printf("Set RIGHT Brightness up error\n");
					break;
				case A_CONTRAST_UP:
					if ((value = v4l2UpControl(videoIn_left, V4L2_CID_CONTRAST)) < 0)
						printf("Set LEFT Contrast up error\n");
					if ((value = v4l2UpControl(videoIn_right, V4L2_CID_CONTRAST)) < 0)
						printf("Set RIGHT Contrast up error\n");

					break;
				case A_SATURATION_UP:
					if ((value = v4l2UpControl(videoIn_left, V4L2_CID_SATURATION)) < 0)
						printf("Set LEFT Saturation up error\n");
					if ((value = v4l2UpControl(videoIn_right, V4L2_CID_SATURATION)) < 0)
						printf("Set RIGHT Saturation up error\n");
					break;
				case A_GAIN_UP:
					if ((value = v4l2UpControl(videoIn_left, V4L2_CID_GAIN)) < 0)
						printf("Set LEFT Gain up error\n");
					if ((value = v4l2UpControl(videoIn_right, V4L2_CID_GAIN)) < 0)
						printf("Set RIGHT Gain up error\n");
					break;
				case A_SHARPNESS_UP:
					if ((value = v4l2UpControl(videoIn_left, V4L2_CID_SHARPNESS)) < 0)
						printf("Set LEFT Sharpness up error\n");
					if ((value = v4l2UpControl(videoIn_right, V4L2_CID_SHARPNESS)) < 0)
						printf("Set RIGHT Sharpness up error\n");
					break;
				case A_GAMMA_UP:
					if ((value = v4l2UpControl(videoIn_left, V4L2_CID_GAMMA)) < 0)
						printf("Set LEFT Gamma up error\n");
					if ((value = v4l2UpControl(videoIn_right, V4L2_CID_GAMMA)) < 0)
						printf("Set RIGHT Gamma up error\n");
					break;

				/* Motor control events */
				case A_PAN_UP:
					if ((value = v4L2UpDownPan(videoIn_left, -incpantilt)) < 0)
						printf("Set LEFT Pan up error\n");
					if ((value = v4L2UpDownPan(videoIn_right, -incpantilt)) < 0)
						printf("Set RIGHT Pan up error\n");

					break;
				case A_PAN_DOWN: 
					if ((value = v4L2UpDownPan(videoIn_left, incpantilt)) < 0)	    
						printf("Set LEFT Pan down error\n");
					if ((value = v4L2UpDownPan(videoIn_right, incpantilt)) < 0)	    
						printf("Set RIGHT Pan down error\n");
					break;
				case A_TILT_UP:
					if ((value = v4L2UpDownTilt(videoIn_left, -incpantilt)) < 0)
						printf("Set LEFT Tilt up error\n");
					if ((value = v4L2UpDownTilt(videoIn_right, -incpantilt)) < 0)
						printf("Set RIGHT Tilt up error\n");
					break;
				case A_TILT_DOWN:
					if ((value = v4L2UpDownTilt(videoIn_left, incpantilt)) < 0)	    
						printf("Set LEFT Tilt down error\n");
					if ((value = v4L2UpDownTilt(videoIn_right, incpantilt)) < 0)	    
						printf("Set RIGHT Tilt down error\n");
					break;
				case A_PAN_RESET:
					if (v4l2ResetPan(videoIn_left) < 0)
						printf("Reset LEFT pan error\n");
					if (v4l2ResetPan(videoIn_right) < 0)
						printf("Reset RIGHT pan error\n");
					break;
				case A_TILT_RESET:
					if (v4l2ResetTilt(videoIn_left) < 0)
						printf("Reset LEFT tilt error\n");
					if (v4l2ResetTilt(videoIn_right) < 0)
						printf("Reset RIGHT tilt error\n");
					break;

				case A_SCREENSHOT:
					SDL_Delay(200);
					videoIn_left->getPict = 1;
					videoIn_right->getPict = 1;
					value = 1;
					break;
				case A_RESET:
					if (v4l2ResetControl(videoIn_left, V4L2_CID_BRIGHTNESS) < 0)
						printf("LEFT reset Brightness error\n");
					if (v4l2ResetControl(videoIn_left, V4L2_CID_SATURATION) < 0)
						printf("LEFT reset Saturation error\n");
					if (v4l2ResetControl(videoIn_left, V4L2_CID_CONTRAST) < 0)
						printf("LEFT reset Contrast error\n");
					if (v4l2ResetControl(videoIn_left, V4L2_CID_HUE) < 0)
						printf("LEFT reset Hue error\n");
					if (v4l2ResetControl(videoIn_left, V4L2_CID_SHARPNESS) < 0)
						printf("LEFT reset Sharpness error\n");
					if (v4l2ResetControl(videoIn_left, V4L2_CID_GAMMA) < 0)
						printf("LEFT reset Gamma error\n");
					if (v4l2ResetControl(videoIn_left, V4L2_CID_GAIN) < 0)
						printf("LEFT reset Gain error\n");
					if (v4l2ResetPanTilt(videoIn_left) < 0)
						printf("LEFT reset pantilt error\n");

					if (v4l2ResetControl(videoIn_right, V4L2_CID_BRIGHTNESS) < 0)
						printf("RIGHT reset Brightness error\n");
					if (v4l2ResetControl(videoIn_right, V4L2_CID_SATURATION) < 0)
						printf("RIGHT reset Saturation error\n");
					if (v4l2ResetControl(videoIn_right, V4L2_CID_CONTRAST) < 0)
						printf("RIGHT reset Contrast error\n");
					if (v4l2ResetControl(videoIn_right, V4L2_CID_HUE) < 0)
						printf("RIGHT reset Hue error\n");
					if (v4l2ResetControl(videoIn_right, V4L2_CID_SHARPNESS) < 0)
						printf("RIGHT reset Sharpness error\n");
					if (v4l2ResetControl(videoIn_right, V4L2_CID_GAMMA) < 0)
						printf("RIGHT reset Gamma error\n");
					if (v4l2ResetControl(videoIn_right, V4L2_CID_GAIN) < 0)
						printf("RIGHT reset Gain error\n");
					if (v4l2ResetPanTilt(videoIn_right) < 0)
						printf("RIGHT reset pantilt error\n");
					break;

				case A_BRIGHTNESS_DOWN:
					if ((value = v4l2DownControl(videoIn_left, V4L2_CID_BRIGHTNESS)) < 0)
						printf("Set LEFT Brightness down error\n");
					if ((value = v4l2DownControl(videoIn_right, V4L2_CID_BRIGHTNESS)) < 0)
						printf("Set RIGHT Brightness down error\n");

					break;
				case A_CONTRAST_DOWN:
					if ((value = v4l2DownControl(videoIn_left, V4L2_CID_CONTRAST)) < 0)
						printf("Set LEFT Contrast down error\n");
					if ((value = v4l2DownControl(videoIn_right, V4L2_CID_CONTRAST)) < 0)
						printf("Set RIGHT Contrast down error\n");
					break;
				case A_SATURATION_DOWN:
					if ((value = v4l2DownControl(videoIn_left, V4L2_CID_SATURATION)) < 0)
						printf("Set LEFT Saturation down error\n");
					if ((value = v4l2DownControl(videoIn_right, V4L2_CID_SATURATION)) < 0)
						printf("Set RIGHT Saturation down error\n");
					break;
				case A_GAIN_DOWN:
					if ((value = v4l2DownControl(videoIn_left, V4L2_CID_GAIN)) < 0)
						printf("Set LEFT Gain down error\n");
					if ((value = v4l2DownControl(videoIn_right, V4L2_CID_GAIN)) < 0)
						printf("Set RIGHT Gain down error\n");
					break;
				case A_SHARPNESS_DOWN:
					if ((value = v4l2DownControl(videoIn_left, V4L2_CID_SHARPNESS)) < 0)
						printf("Set LEFT Sharpness down error\n");
					if ((value = v4l2DownControl(videoIn_right, V4L2_CID_SHARPNESS)) < 0)
						printf("Set RIGHT Sharpness down error\n");
					break;
				case A_GAMMA_DOWN:
					if ((value = v4l2DownControl(videoIn_left, V4L2_CID_GAMMA)) < 0)
						printf("Set LEFT Gamma down error\n");
					if ((value = v4l2DownControl(videoIn_right, V4L2_CID_GAMMA)) < 0)
						printf("Set RIGHT Gamma down error\n");
					break;   
				case A_RECORD_TOGGLE:
					SDL_Delay(200);
					videoIn_left->toggleAvi = !videoIn_left->toggleAvi;
					value = videoIn_left->toggleAvi;
					if ( value == 1 ) {
						printf("LEFT avi recording started\n");
						videoIn_left->recordstart=SDL_GetTicks();
					}
					else {
						int dur=SDL_GetTicks()-videoIn_left->recordstart;
						printf("\nLEFT avi recording stopped (%ds)\n",dur/1000);
						videoIn_left->recordtime+=dur;
					}

					videoIn_right->toggleAvi = !videoIn_right->toggleAvi;
					value = videoIn_right->toggleAvi;
					if ( value == 1 ) {
						printf("RIGHT avi recording started\n");
						videoIn_right->recordstart=SDL_GetTicks();
					}
					else {
						int dur=SDL_GetTicks()-videoIn_right->recordstart;
						printf("\nRIGHT avi recording stopped (%ds)\n",dur/1000);
						videoIn_right->recordtime+=dur;
					}
					break;
				case A_SWITCH_LIGHTFREQFILT:
					if ((value =v4l2GetControl(videoIn_left,V4L2_CID_POWER_LINE_FREQUENCY)) < 0)	    
						printf("LEFT Get value of light frequency filter error\n");

					if(value < 2) // round switch 50->60->NoFliker->.
						value++;   //		 \_______________; 
					else
						value=0;

					if(value == 0)
						printf("LEFT Current light frequency filter: 50Hz\n");
					else if(value == 1)
						printf("LEFT Current light frequency filter: 60Hz\n");
					else if(value == 2)
						printf("LEFT Current light frequency filter: NoFliker\n");

					if ((value =v4l2SetLightFrequencyFilter(videoIn_left,value)) < 0)	    
						printf("LEFT Switch light frequency filter error\n");

					if ((value =v4l2GetControl(videoIn_right,V4L2_CID_POWER_LINE_FREQUENCY)) < 0)	    
						printf("RIGHT Get value of light frequency filter error\n");

					if(value < 2) // round switch 50->60->NoFliker->.
						value++;   //		 \_______________; 
					else
						value=0;

					if(value == 0)
						printf("RIGHT Current light frequency filter: 50Hz\n");
					else if(value == 1)
						printf("RIGHT Current light frequency filter: 60Hz\n");
					else if(value == 2)
						printf("RIGHT Current light frequency filter: NoFliker\n");

					if ((value =v4l2SetLightFrequencyFilter(videoIn_right,value)) < 0)	    
						printf("RIGHT Switch light frequency filter error\n");
					break;
				case A_QUIT:
					videoIn_left->signalquit = 0;
					videoIn_right->signalquit = 0;
					break;
				case A_VIDEO:
					break;
				case A_DEBUG:
					post_screen_flag = 1;
					break;
				case A_CAPTURE_FRAME:
					value = 1;
					videoIn_left->rawFrameCapture = 1;
					videoIn_right->rawFrameCapture = 1;
					break;
				case A_CAPTURE_FRAMESTREAM:
					value = 1;
					if (!videoIn_left->rawFrameCapture) {
						videoIn_left->rawFrameCapture = 2;
						videoIn_left->rfsBytesWritten = 0;
						videoIn_left->rfsFramesWritten = 0;
						printf("LEFT Starting raw frame stream capturing ...\n");
					} else if(videoIn_left->framesWritten >= 5) {
						videoIn_left->rawFrameCapture = 0;
						printf("LEFT Stopped raw frame stream capturing. %u bytes written for %u frames.\n",
								videoIn_left->rfsBytesWritten, videoIn_left->rfsFramesWritten);
					}

					if (!videoIn_right->rawFrameCapture) {
						videoIn_right->rawFrameCapture = 2;
						videoIn_right->rfsBytesWritten = 0;
						videoIn_right->rfsFramesWritten = 0;
						printf("RIGHT Starting raw frame stream capturing ...\n");
					} else if(videoIn_right->framesWritten >= 5) {
						videoIn_right->rawFrameCapture = 0;
						printf("RIGHT Stopped raw frame stream capturing. %u bytes written for %u frames.\n",
								videoIn_right->rfsBytesWritten, videoIn_right->rfsFramesWritten);
					}
					break;
				case A_CAPTURE_STREAM:
					value = 1;
					if (videoIn_left->captureFile == NULL) {
						videoIn_left->captureFile = fopen("stream_left.raw", "wb");
						if(videoIn_left->captureFile == NULL) {
							perror("LEFT Unable to open file for raw stream capturing");
						} else {
							printf("LEFT Starting raw stream capturing to stream.raw ...\n");
						}
						videoIn_left->bytesWritten = 0;
						videoIn_left->framesWritten = 0;
					} else if(videoIn_left->framesWritten >= 5) {
						fclose(videoIn_left->captureFile);
						printf("LEFT Stopped raw stream capturing to stream.raw. %u bytes written for %u frames.\n",
								videoIn_left->bytesWritten, videoIn_left->framesWritten);
						videoIn_left->captureFile = NULL;
					}

					if (videoIn_right->captureFile == NULL) {
						videoIn_right->captureFile = fopen("stream_right.raw", "wb");
						if(videoIn_right->captureFile == NULL) {
							perror("RIGHT Unable to open file for raw stream capturing");
						} else {
							printf("RIGHT Starting raw stream capturing to stream.raw ...\n");
						}
						videoIn_right->bytesWritten = 0;
						videoIn_right->framesWritten = 0;
					} else if(videoIn_right->framesWritten >= 5) {
						fclose(videoIn_right->captureFile);
						printf("RIGHT Stopped raw stream capturing to stream.raw. %u bytes written for %u frames.\n",
								videoIn_right->bytesWritten, videoIn_right->framesWritten);
						videoIn_right->captureFile = NULL;
					}
					break;
				case A_EXPOSURE_UP:
					if ((value = v4l2UpControl(videoIn_left, V4L2_CID_EXPOSURE_ABSOLUTE)) < 0)
						printf("LEFT Set Absolute Exposure up error\n");
					if ((value = v4l2UpControl(videoIn_right, V4L2_CID_EXPOSURE_ABSOLUTE)) < 0)
						printf("RIGHT Set Absolute Exposure up error\n");
					break;
				case A_EXPOSURE_DOWN:
					if ((value = v4l2DownControl(videoIn_left, V4L2_CID_EXPOSURE_ABSOLUTE)) < 0)
						printf("LEFT Set Absolute Exposure down error\n");
					if ((value = v4l2DownControl(videoIn_right, V4L2_CID_EXPOSURE_ABSOLUTE)) < 0)
						printf("RIGHT Set Absolute Exposure down error\n");
					break;
				case A_EXPOSURE_ON:
					control.id    =V4L2_CID_EXPOSURE_AUTO;
					control.value =V4L2_EXPOSURE_AUTO;
					if ((value = ioctl(videoIn_left->fd, VIDIOC_S_CTRL, &control)) < 0)
						printf("LEFT Set Auto Exposure on error\n");
					else
						printf("LEFT Auto Exposure set to %d\n", control.value);

					if ((value = ioctl(videoIn_right->fd, VIDIOC_S_CTRL, &control)) < 0)
						printf("RIGHT Set Auto Exposure on error\n");
					else
						printf("RIGHT Auto Exposure set to %d\n", control.value);
					break;
				case A_EXPOSURE_OFF:
					control.id    =V4L2_CID_EXPOSURE_AUTO;
					control.value =V4L2_EXPOSURE_MANUAL;
					if ((value = ioctl(videoIn_left->fd, VIDIOC_S_CTRL, &control)) < 0)
						printf("LEFT Set Auto Exposure off error\n");
					else
						printf("LEFT Auto Exposure set to %d\n", control.value);
					if ((value = ioctl(videoIn_right->fd, VIDIOC_S_CTRL, &control)) < 0)
						printf("RIGHT Set Auto Exposure off error\n");
					else
						printf("RIGHT Auto Exposure set to %d\n", control.value);
					break;
				case A_BALANCE_UP:
					if ((value = v4l2UpControl(videoIn_left, V4L2_CID_WHITE_BALANCE_TEMPERATURE)) < 0)
						printf("LEFT Set Balance Temperature up error\n");
					if ((value = v4l2UpControl(videoIn_right, V4L2_CID_WHITE_BALANCE_TEMPERATURE)) < 0)
						printf("RIGHT Set Balance Temperature up error\n");
					break;
				case A_BALANCE_DOWN:
					if ((value = v4l2DownControl(videoIn_left, V4L2_CID_WHITE_BALANCE_TEMPERATURE)) < 0)
						printf("LEFT Set Balance Temperature down error\n");
					if ((value = v4l2DownControl(videoIn_right, V4L2_CID_WHITE_BALANCE_TEMPERATURE)) < 0)
						printf("RIGHT Set Balance Temperature down error\n");
					break;
				case A_BALANCE_ON:
					control.id    =V4L2_CID_AUTO_WHITE_BALANCE;
					control.value =1;
					if ((value = ioctl(videoIn_left->fd, VIDIOC_S_CTRL, &control)) < 0)
						printf("LEFT Set Auto Balance on error\n");
					else
						printf("LEFT Auto Balance set to %d\n", control.value);

					if ((value = ioctl(videoIn_right->fd, VIDIOC_S_CTRL, &control)) < 0)
						printf("RIGHT Set Auto Balance on error\n");
					else
						printf("RIGHT Auto Balance set to %d\n", control.value);

					break;
				case A_BALANCE_OFF:
					control.id    =V4L2_CID_AUTO_WHITE_BALANCE;
					control.value =0;
					if ((value = ioctl(videoIn_left->fd, VIDIOC_S_CTRL, &control)) < 0)
						printf("LEFT Set Auto Balance off error\n");
					else
						printf("LEFT Auto Balance set to %d\n", control.value);

					if ((value = ioctl(videoIn_right->fd, VIDIOC_S_CTRL, &control)) < 0)
						printf("RIGHT Set Auto Balance off error\n");
					else
						printf("RIGHT Auto Balance set to %d\n", control.value);
					break;
				case A_SAVE:
					printf("Save controls\n");
					save_controls(videoIn_left->fd);
					//save_controls(videoIn_right->fd);
					break;
				case A_LOAD:
					printf("load controls\n");
					load_controls(videoIn_left->fd);
					load_controls(videoIn_right->fd);
					break;
				default:
					break;
			}
			if(!(boucle%10)) // smooth pan tilt method
				if(incpantilt < (10*INCPANTILT))
					incpantilt += (INCPANTILT/4);
			if(value){
				len = strlen(title_act[curr_action].title)+8;
				snprintf(videoIn_left->status, len,"%s %06d",title_act[curr_action].title,value);
			}
		} else { // mouseon
			len = 100 * sizeof(char);	// as allocated in init_videoIn
			snprintf(videoIn_left->status, len, "%s, %.1f fps", title_act[curr_action].title, frmrate);
		}
		SDL_Delay(50);
		//printf("fp/s %d\n",frmrate);
	}				//end main loop

	/* Close the stream capture file */
	if (videoIn_left->captureFile) {
		fclose(videoIn_left->captureFile);
		printf("Stopped raw stream capturing to stream.raw. %u bytes written for %u frames.\n",
				videoIn_left->bytesWritten, videoIn_left->framesWritten);
	}

	if (videoIn_right->captureFile) {
		fclose(videoIn_right->captureFile);
		printf("Stopped raw stream capturing to stream.raw. %u bytes written for %u frames.\n",
				videoIn_right->bytesWritten, videoIn_right->framesWritten);
	}
	/* Display stats for raw frame stream capturing */
	if (videoIn_left->rawFrameCapture == 2) {
		printf("Stopped raw frame stream capturing. %u bytes written for %u frames.\n",
				videoIn_left->rfsBytesWritten, videoIn_left->rfsFramesWritten);
	}
	if (videoIn_right->rawFrameCapture == 2) {
		printf("Stopped raw frame stream capturing. %u bytes written for %u frames.\n",
				videoIn_right->rfsBytesWritten, videoIn_right->rfsFramesWritten);
	}
}

static int eventthread_camera_left(void *data)
{
	int i = 0;
	struct pt_camera_data *gdata = (struct pt_camera_data *)data;
	SDL_Overlay *poverlay = *gdata->ptoverlay;
	struct vdIn *videoIn = gdata->ptvideoIn;
	SDL_Rect *drect = gdata->drect;
	SDL_mutex *affmutex = gdata->affmutex;
	SDL_mutex *trackretex = gdata->trackretex;
	unsigned char *p = (unsigned char *)poverlay->pixels[0];

	while (videoIn->signalquit) {
		if (left_time_start == -1) {
		   left_time_start = vcos_getmicrosecs64_internal()/1000;
		}

		SDL_LockMutex(trackretex);
		for (i=0; i<NB_BUFFER; i++) {
			if(videoIn->framebuffer_state[i] == BUFFER_FREE) {
				videoIn->framebuffer_state[i] = CAPTURE_USING;
				break;
			}
		}
		SDL_UnlockMutex(trackretex);

		if ( i != NB_BUFFER ) {
			if (uvcGrab_left_ycbcr(videoIn, i)/*uvcGrab_left(videoIn, i)*/ < 0) {
				printf("Error grabbing\n");
				break;
			} else {
				total_frames_left++;
				SDL_LockMutex(trackretex);
				videoIn->framebuffer_state[i] = CAPTURE_USED;
				videoIn->latest_buffer_number = i;
				SDL_UnlockMutex(trackretex);
			}

			if (post_screen_flag) {
				SDL_LockMutex(affmutex);
				SDL_LockYUVOverlay(poverlay);
				memcpy(p, videoIn->framebuffer[i],
						videoIn->width * (videoIn->height) * 2);
				SDL_UnlockYUVOverlay(poverlay);
				SDL_DisplayYUVOverlay(poverlay, drect);
				SDL_UnlockMutex(affmutex);
			}
		}else {
			//printf("left not be used error\n");
		}
#if 0
		if (videoIn->getPict) {
			switch(videoIn->formatIn){
				case V4L2_PIX_FMT_MJPEG:
					get_picture_left(videoIn->tmpbuffer[i], videoIn->buf.bytesused);
					break;
				case V4L2_PIX_FMT_YUYV:
					get_pictureYV2_Left(videoIn->framebuffer[i], videoIn->width, videoIn->height);
					break;
				default:
					break;
			}
			videoIn->getPict = 0;
			printf("get picture !\n");
		}
#endif
	}
}

static int eventthread_camera_right(void *data)
{
	int i = 0;
	struct pt_camera_data *gdata = (struct pt_camera_data *)data;
	SDL_Overlay *poverlay = *gdata->ptoverlay;
	struct vdIn *videoIn = gdata->ptvideoIn;
	SDL_Rect *drect = gdata->drect;
	SDL_mutex *affmutex = gdata->affmutex;
	SDL_mutex *trackretex = gdata->trackretex;
	unsigned char *p = (unsigned char *)poverlay->pixels[0];

	while (videoIn->signalquit) {
		if (right_time_start == -1) {
		   right_time_start = vcos_getmicrosecs64_internal()/1000;
		}

		SDL_LockMutex(trackretex);
		for (i=0; i<NB_BUFFER; i++) {
			if(videoIn->framebuffer_state[i]==BUFFER_FREE){
				videoIn->framebuffer_state[i] = CAPTURE_USING;
				break;
			}
		}
		SDL_UnlockMutex(trackretex);

		if ( i != NB_BUFFER ) {
			if (uvcGrab_right_ycbcr(videoIn, i) < 0) {
				printf("Error grabbing\n");
				break;
			} else {
				total_frames_right++;
				SDL_LockMutex(trackretex);
				videoIn->framebuffer_state[i] = CAPTURE_USED;
				videoIn->latest_buffer_number = i;
				SDL_UnlockMutex(trackretex);
			}

			if (post_screen_flag) {
				SDL_LockMutex(affmutex);
				SDL_LockYUVOverlay(poverlay);
				memcpy(p, videoIn->framebuffer[i],
						videoIn->width * (videoIn->height) * 2);
				SDL_UnlockYUVOverlay(poverlay);
				SDL_DisplayYUVOverlay(poverlay, drect);
				SDL_UnlockMutex(affmutex);
			}
		}else {
			//printf("right not be used\n");
		}
#if 0
		if (videoIn->getPict) {
			switch(videoIn->formatIn){
				case V4L2_PIX_FMT_MJPEG:
					get_picture_right(videoIn->tmpbuffer[i],videoIn->buf.bytesused);
					break;
				case V4L2_PIX_FMT_YUYV:
					get_pictureYV2_Right(videoIn->framebuffer[i],videoIn->width,videoIn->height);
					break;
				default:
					break;
			}
			videoIn->getPict = 0;
			printf("get picture !\n");
		}
#endif
	}
}

void call_trackre(void)
{
	//TODO
	;
}

static int eventthread_trackre(void *data)
{
	int i = 0;
	static int number_picture = 0;
	static int left_buffer_number = -1;
	static int right_buffer_number = -1;
	static int left_flag = 0;
	static int right_flag = 0;
	struct pt_trackre_data *gdata = (struct pt_trackre_data *)data;
	struct vdIn *videoIn_left = gdata->ptvideoIn_left;
	struct vdIn *videoIn_right = gdata->ptvideoIn_right;
	SDL_mutex *trackretex_left = gdata->trackretex_left;
	SDL_mutex *trackretex_right = gdata->trackretex_right;

	if (trackre_time_start == -1) {
	   trackre_time_start = vcos_getmicrosecs64_internal()/1000;
	}

	while (videoIn_left->signalquit && videoIn_right->signalquit) {
		SDL_LockMutex(trackretex_left);
		if (((videoIn_left->latest_buffer_number>=0) && (videoIn_left->latest_buffer_number<=3)) &&
				(videoIn_left->framebuffer_state[videoIn_left->latest_buffer_number] == CAPTURE_USED)) {
			left_buffer_number = videoIn_left->latest_buffer_number;
			left_flag = 1;
		} else {
			left_flag = 0;
		}
		SDL_UnlockMutex(trackretex_left);

		SDL_LockMutex(trackretex_right);
		if (((videoIn_left->latest_buffer_number>=0) && (videoIn_left->latest_buffer_number<=3)) &&
				(videoIn_right->framebuffer_state[videoIn_right->latest_buffer_number] == CAPTURE_USED)) {
			right_buffer_number = videoIn_right->latest_buffer_number;
			right_flag = 1;
		} else {
			right_flag = 0;
		}
		SDL_UnlockMutex(trackretex_right);

		if (left_flag && right_flag) {
			SDL_LockMutex(trackretex_left);
			if (videoIn_left->getPict) {
				switch(videoIn_left->formatIn){
					case V4L2_PIX_FMT_MJPEG:
						//get_picture_left(videoIn_left->tmpbuffer[left_buffer_number], videoIn_left->buf_used[left_buffer_number], number_picture);
						//get_bmp_picture_left(videoIn_left->rgbbuffer[left_buffer_number], number_picture);
						//get_gray_picture_left(videoIn_left->graybuffer[left_buffer_number], number_picture);
						get_yuv_picture_left(videoIn_left->graybuffer[left_buffer_number], videoIn_left->cbbuffer[left_buffer_number],
												videoIn_left->crbuffer[left_buffer_number], number_picture);
						break;
					case V4L2_PIX_FMT_YUYV:
						get_pictureYV2_Left(videoIn_left->framebuffer[left_buffer_number], videoIn_left->width, videoIn_left->height);
						break;
					default:
						break;
				}
				videoIn_left->getPict = 0;
				printf("Left get picture !\n");
			}
			SDL_UnlockMutex(trackretex_left);

			SDL_LockMutex(trackretex_right);
			if (videoIn_right->getPict) {
				switch(videoIn_right->formatIn){
					case V4L2_PIX_FMT_MJPEG:
						//get_picture_right(videoIn_right->tmpbuffer[right_buffer_number], videoIn_right->buf_used[right_buffer_number], number_picture);
						//get_bmp_picture_right(videoIn_right->rgbbuffer[right_buffer_number], number_picture);
						//get_gray_picture_right(videoIn_right->graybuffer[right_buffer_number], number_picture);
						get_yuv_picture_right(videoIn_right->graybuffer[right_buffer_number], videoIn_right->cbbuffer[right_buffer_number],
												videoIn_right->crbuffer[right_buffer_number], number_picture);
						break;
					case V4L2_PIX_FMT_YUYV:
						get_pictureYV2_Right(videoIn_right->framebuffer[right_buffer_number], videoIn_right->width, videoIn_right->height);
						break;
					default:
						break;
				}
				videoIn_right->getPict = 0;
				number_picture++;
				printf("Right get picture !\n");
			}
			SDL_UnlockMutex(trackretex_right);
		}

		if (left_flag && right_flag) {
			call_trackre();
			total_frames_trackre++;
			SDL_LockMutex(trackretex_left);
			for (i=0; i<NB_BUFFER; i++) {
				if (videoIn_left->framebuffer_state[i] == CAPTURE_USED && 
						videoIn_left->latest_buffer_number != i) {
					videoIn_left->framebuffer_state[i] = BUFFER_FREE;
				}
			}
			videoIn_left->framebuffer_state[left_buffer_number] = BUFFER_FREE;
			SDL_UnlockMutex(trackretex_left);

			SDL_LockMutex(trackretex_right);
			for (i=0; i<NB_BUFFER; i++) {
				if (videoIn_right->framebuffer_state[i] == CAPTURE_USED && 
						videoIn_right->latest_buffer_number != i) {
					videoIn_right->framebuffer_state[i] = BUFFER_FREE;
				}
			}
			videoIn_right->framebuffer_state[right_buffer_number] = BUFFER_FREE;
			SDL_UnlockMutex(trackretex_right);
		}
	}
}
