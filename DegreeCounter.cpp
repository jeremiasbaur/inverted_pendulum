/*#include "DegreeCounter.h"

DegreeCounter::DegreeCounter(int interruptA, int interruptB) {
  m_interruptA = interruptA;
  m_interruptB = interruptB;
  mux = portMUX_INITIALIZER_UNLOCKED;
  pinMode(m_interruptA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m_interruptA), handleInterruptA, CHANGE);
}

void DegreeCounter::ChangeDegrees() {
  // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (stateA != old_stateA) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(m_interruptB) != aState) {
      counter++;
    } else {
      counter--;
    }
    //Serial.print("Position: ");
    //Serial.println(counter);
  }
  old_stateA = stateA; // Updates the previous state of the outputA with the current state
}

void DegreeCounter::IRAM_ATTR handleInterruptA() {
  portENTER_CRITICAL_ISR(&amp; mux);
  //interruptCounter++;
  stateA = !stateA;
  ChangeDegrees();
  portEXIT_CRITICAL_ISR(&amp; mux);
}

int DegreeCounter::ReturnDegrees() {
  return counter;
}*/
