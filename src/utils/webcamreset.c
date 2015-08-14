/**
 * \file	webcamreset.c
 * \author	Karl Ritchie <ritchie.karl@gmail.com>
 * \date	28/12/2014
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

//==============================================================================
// I N C L U D E   F I L E S

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <linux/usbdevice_fs.h>

int main(int argc, char **argv) {
  const char *filename;
  int fd;
  int rc;

  if (argc != 2) {
    fprintf(stderr, "Usage: usbreset device-filename\n");
    return 1;
  }
  filename = argv[1];

  fd = open(filename, O_WRONLY);
  if (fd < 0) {
    perror("Error opening output file");
    return 1;
  }

  printf("Resetting USB device %s\n", filename);
  rc = ioctl(fd, USBDEVFS_RESET, 0);
  if (rc < 0) {
    perror("Error in ioctl");
    return 1;
  }
  printf("Reset successful\n");

  close(fd);
  return 0;
}
