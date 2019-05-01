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
    int *position_array;
    int *time_array;
    int array_length;
  public:
    PVA(volatile int* position_pointer, int array_length);
    void newPosition();
    int getPosition();
    double getVelocity();
    double getAccleration();
    double getAbsVelocity();
    int getModPosition(int mod);
		bool noMovementWaiter();
};
