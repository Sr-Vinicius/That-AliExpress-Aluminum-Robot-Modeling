#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <unistd.h>
#include "network/SocketServer.h"
#include "pca9685_rasp.h"

#define MG996R_MIN_POSITION  1*(4096*50/1000)
#define MG996R_MID_POSITION  1.5*(4096*50/1000)
#define MG996R_MAX_POSITION  2*(4096*50/1000)
#define MG996R_RAD_TO_PWM(a) (round(307.2/M_PI * a + 204.8))
#define MG996R_MIN_STEP      30000 /* microsseconds */

using namespace exploringRPi;

int main(int argc, char *argv[]){
  /* PCA9685 servo driver setup */
  pca9685 servo_driver(1, 0x43);
  servo_driver.set_output_drive(pca9685::TOTEM_POLE);
  servo_driver.set_output_inverting(false);
  servo_driver.set_output_enable_pin(26);
  servo_driver.set_output_enable_value(LOW);
  servo_driver.set_pwm_frequency(50);

  /* Setting servo signal channels*/
  servo_driver.enable_channel(pca9685::CH00);
  servo_driver.enable_channel(pca9685::CH01);
  servo_driver.enable_channel(pca9685::CH02);
  servo_driver.enable_channel(pca9685::CH03);
  servo_driver.enable_channel(pca9685::CH04);

  /* Going to zero position */
  servo_driver.set_pwm_duty_cycle(pca9685::CH00, MG996R_MIN_POSITION);
  servo_driver.set_pwm_duty_cycle(pca9685::CH01, MG996R_MIN_POSITION);
  servo_driver.set_pwm_duty_cycle(pca9685::CH02, MG996R_MIN_POSITION);
  servo_driver.set_pwm_duty_cycle(pca9685::CH03, MG996R_MIN_POSITION);
  servo_driver.set_pwm_duty_cycle(pca9685::CH04, MG996R_MIN_POSITION);
  sleep(5);

    std::vector< float > joints;
    std::string aux;
    std::cout << "Starting RPi Server Example" << std::endl;
    SocketServer server(54321);
    std::cout << "Listening for a connection" << std::endl;
    server.listen();
    
    while(true){
      std::string rec = server.receive(1024);
      unsigned int last_space_pos = 0;
      for(int i=0; i<rec.length(); i++){
        if(rec.at(i) == ' '){
          if(i - last_space_pos > 1){
            joints.push_back( std::stof(aux) );
            aux.clear();
          }
          last_space_pos = i;
        }
        else
          aux.push_back(rec.at(i));
      }
      joints.push_back( std::stof(aux) );
      for(std::vector<float>::iterator it=joints.begin(), pca9685::CHANNEL ch = pca9685::CH00;
          it!=joints.end();
          it++, ch++){
        std::cout << *it << " ";
        servo_driver.set_pwm_duty_cycle(ch, *it);       
      }
      aux.clear();
      joints.clear();
      usleep(30000);
      std::cout << "\n\n";
    }

    return 0;
}
