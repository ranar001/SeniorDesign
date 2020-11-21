#include "rims.h"

/*Define user variables and functions for this state machine here.*/
unsigned char SM1_Clk;
void TimerISR() {
   SM1_Clk = 1;
}

unsigned char SM1_rx_flag = 0;
void RxISR() {
   SM1_rx_flag = 1;
}

enum SM1_States { SM1_stop, SM1_moveRT, SM1_moveLFT, SM1_moveFW, SM1_moveBW } SM1_State;

TickFct_State_machine_1() {
   switch(SM1_State) { // Transitions
      case -1:
         SM1_State = SM1_stop;
         break;
         case SM1_stop: 
         if (!A1 && A2) {
            SM1_State = SM1_moveRT;
         }
         else if (A1 && !A2 ) {
            SM1_State = SM1_moveLFT;
         }
         else if (A0 && !A3) {
            SM1_State = SM1_moveFW;
         }
         else if (!A0 && A3) {
            SM1_State = SM1_moveBW;
         }
         break;
      case SM1_moveRT: 
         if (!A1 && A2) {
            SM1_State = SM1_moveRT;
         }
         else if (A1 && !A2) {
            SM1_State = SM1_moveLFT;
         }
         else if ((A1 && A2) || (!A1 && !A2)) {
            SM1_State = SM1_stop;
         }
         break;
      case SM1_moveLFT: 
         if (A1 && !A2 ) {
            SM1_State = SM1_moveLFT;
         }
         else if (!A1 && A2) {
            SM1_State = SM1_moveRT;
         }
         else if ((A1 && A2) || (!A1 && !A2)) {
            SM1_State = SM1_stop;
         }
         break;
      case SM1_moveFW: 
         if (A0 && !A3) {
            SM1_State = SM1_moveFW;
         }
         else if (!A0 && A3) {
            SM1_State = SM1_moveBW;
         }
         else if ((!A0 && !A3) || (A0 && A3)) {
            SM1_State = SM1_stop;
         }
         break;
      case SM1_moveBW: 
         if (!A0 && A3) {
            SM1_State = SM1_moveBW;
         }
         else if (A0 && !A3) {
            SM1_State = SM1_moveFW;
         }
         else if ((!A0 && !A3) || (A0 && A3)) {
            SM1_State = SM1_stop;
         }
         break;
      default:
         SM1_State = SM1_stop;
   } // Transitions

   switch(SM1_State) { // State actions
//MUST EDIT THIS SECTION TO COMMUNICATE WITH
//THE MOTORS MOUNTED
      case SM1_stop:
         break;
      case SM1_moveRT:
         break;
      case SM1_moveLFT:
         break;
      case SM1_moveFW:
         break;
      case SM1_moveBW:
         break;
      default: 
      break;
   } // State actions

}

int main() {

   const unsigned int periodState_machine_1 = 500;
   TimerSet(periodState_machine_1);
   TimerOn();
   UARTOn();


   SM1_State = -1; // Initial state
   B = 0; // Init outputs

   while(1) {
      TickFct_State_machine_1();
      while(!SM1_Clk);
      SM1_Clk = 0;
   } // while (1)
} // Main

