class PVA
{
  private:
    volatile int *position_pointer;
    unsigned long time_now, time_last;
    int last_position;
    double current_velocity;
    double last_velocity;
    double current_accleration;
    void setVelocity();
    void setAccleration();
  public:
    PVA(volatile int* position_pointer);
    void newPosition();
    int getPosition();
    double getVelocity();
    double getAccleration();
};
