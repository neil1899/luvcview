/*******************************************************************************
#	 	luvcview: Sdl video Usb Video Class grabber          .         #
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
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include "avilib.h"

#include "uvcvideo.h"
#include "dynctrl-logitech.h"

//for libjpeg
#include <setjmp.h>
#include <jpeglib.h>

#define NB_BUFFER 4
#define DHT_SIZE 432

#define PUT_2B(array,offset,value)  \
        (array[offset]   = (char) ((value) & 0xFF), \
         array[offset+1] = (char) (((value) >> 8) & 0xFF))

#define PUT_4B(array,offset,value)  \
        (array[offset]   = (char) ((value) & 0xFF), \
         array[offset+1] = (char) (((value) >> 8) & 0xFF), \
         array[offset+2] = (char) (((value) >> 16) & 0xFF), \
         array[offset+3] = (char) (((value) >> 24) & 0xFF))

typedef enum buffer_state {
    BUFFER_FREE,
    CAPTURE_USING,
    CAPTURE_USED,
    MAX,
} buffer_state;

struct error_mgr_t {
	struct jpeg_error_mgr pub;    /* "public" fields */
	jmp_buf setjmp_buffer;    /* for return to caller */
};

typedef struct error_mgr_t * error_ptr;

struct vdIn {
    int fd;
    char *videodevice;
    char *status;
    char *pictName;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers rb;
	struct jpeg_decompress_struct cinfo;
    struct error_mgr_t jerr;
    void *mem[NB_BUFFER];
    unsigned char *tmpbuffer[NB_BUFFER];
    unsigned char *framebuffer[NB_BUFFER];
	unsigned char *rgbbuffer[NB_BUFFER];
	unsigned char *graybuffer[NB_BUFFER];
	unsigned char *cbbuffer[NB_BUFFER];
	unsigned char *crbuffer[NB_BUFFER];
	int framebuffer_state[NB_BUFFER];
	int buf_used[NB_BUFFER];
	int latest_buffer_number;
	int isjpeg;
    int isstreaming;
    int grabmethod;
    int width;
    int height;
    float fps;
    int formatIn;
    int formatOut;
    int framesizeIn;
    int signalquit;
    int toggleAvi;
    int getPict;
    int rawFrameCapture;
    /* raw frame capture */
    unsigned int fileCounter;
    /* raw frame stream capture */
    unsigned int rfsFramesWritten;
    unsigned int rfsBytesWritten;
    /* raw stream capture */
    FILE *captureFile;
    unsigned int framesWritten;
    unsigned int bytesWritten;
    avi_t *avifile;
    char *avifilename;
    int framecount;
    int recordstart;
    int recordtime;
};

int
init_videoIn(struct vdIn *vd, char *device, int width, int height, float fps,
	     int format, int grabmethod, char *avifilename);
int enum_controls(int vd);
int save_controls(int vd);
int load_controls(int vd);
	     
int uvcGrab_left(struct vdIn *vd, int buffer_number);
int uvcGrab_right(struct vdIn *vd, int buffer_number);
int uvcGrab_left_rgb(struct vdIn *vd, int buffer_number);
int uvcGrab_right_rgb(struct vdIn *vd, int buffer_number);

int get_bmp_picture_left(unsigned char *buf, int number);
int get_bmp_picture_right(unsigned char *buf, int number);

int close_v4l2(struct vdIn *vd);

int v4l2GetControl(struct vdIn *vd, int control);
int v4l2SetControl(struct vdIn *vd, int control, int value);
int v4l2UpControl(struct vdIn *vd, int control);
int v4l2DownControl(struct vdIn *vd, int control);
int v4l2ToggleControl(struct vdIn *vd, int control);
int v4l2ResetControl(struct vdIn *vd, int control);
int v4l2ResetPanTilt(struct vdIn *vd);
int v4L2UpDownPan(struct vdIn *vd, short inc);
int v4L2UpDownTilt(struct vdIn *vd,short inc);
int v4L2UpDownPanTilt(struct vdIn *vd, short inc_p, short inc_t);
int v4l2SetLightFrequencyFilter(struct vdIn *vd,int flt);
int enum_frame_intervals(int dev, __u32 pixfmt, __u32 width, __u32 height);
int enum_frame_sizes(int dev, __u32 pixfmt);
int enum_frame_formats(int dev, unsigned int *supported_formats, unsigned int max_formats);
