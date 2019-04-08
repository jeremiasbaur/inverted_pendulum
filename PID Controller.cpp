//PID Control
/*#include <vector>
#include <iostream>

using namespace std;

long long t_step = 0;
//gains
double propo = 1;
double deriv = 1;
double integ = 1;

//used to filter noise
//and make integral term not last too long
double filter_deriv = 20;
double filter_integ = 100;
vector<double> avg_error (filter_deriv);
vector<double> sum_error (filter_integ);
double curr_avg = 0;
double curr_sum = 0;

//input angle -90 to 90
//output number -1 to 1
double PID(double angle){
    angle /= 90;
    double ans;

    curr_avg -= (avg_error[t_step % (int)filter_deriv]/ filter_deriv);
    curr_avg += (angle/filter_deriv);
    avg_error[t_step % (int)filter_deriv] = angle;

    curr_sum -= (sum_error[t_step % (int)filter_integ]/ filter_integ);
    curr_sum += (angle/filter_integ);
    sum_error[t_step % (int)filter_integ] = angle;

    ans = (angle * propo) + ((angle - curr_avg) * deriv) + ((angle - curr_sum) * integ);
    cout << (angle * propo) << ' ' << ((angle - curr_avg) * deriv) << ' ' << ((angle + curr_sum) * integ) << '\n';
    ans /= (propo + deriv + integ);
    t_step += 1;
    return ans;
}

int main(){
    double a, b;
    while(a != 100){
        cin >> a;
        b = PID(a);
        cout << b;
    }
    return 1;
}*/
