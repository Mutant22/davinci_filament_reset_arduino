/*
Main program for raspberry filament reseter.
*/
#define RPI 1
#include "xyz_dv_eprom.h"

int main(int argc, char** argv)
{
  /*
  TODO:
       - set temp, length, ... by argument
  */
  setup_io();
  loop();
}
