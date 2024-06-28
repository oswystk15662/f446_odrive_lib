#include <mbed.h>

#include "f446_odrive_can.hpp"

using namespace osw_no_heya;

odrive_settings settings = {

};
f446_odrive_can can(D15, D14, 0x01, settings);

int main() {

  // put your setup code here, to run once:

  while(1) {
    // put your main code here, to run repeatedly:
  }
}