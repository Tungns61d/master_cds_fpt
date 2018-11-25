#include "utils.h"
#include <iostream>
using namespace std;
using namespace cv;

/*--------------------------------------------------------------------
* Function: Enqueue
* Purpose: create data node, add into queue linkedlist
* In arg: cv::Mat
* In/out arg: queue_head, queue_tail
*/
void
Enqueue(cv::Mat &img,
        list_node_s** queue_head,
        list_node_s** queue_tail,
        int &queue_size)
{
    list_node_s* tmp_node = NULL;

    tmp_node = new list_node_s();
    tmp_node->data = img.clone();
    tmp_node->next = NULL;

    if (*queue_tail == NULL)  // list is empty
    {
        *queue_head = tmp_node;
        *queue_tail = tmp_node;
    }
    else
    {
        (*queue_tail)->next = tmp_node;
        *queue_tail = tmp_node;
    }
    queue_size++;
}


/*--------------------------------------------------------------------
* Function: Dequeue
* Purpose: remove a node from queue linkedlist and tokenize them
* In/out arg: queue_head, queue_tail
* Ret val: Node at head of queue, or NULL if queue is empty
*/

list_node_s*
Dequeue(list_node_s** queue_head,
        list_node_s** queue_tail,
        int &queue_size)
{

    list_node_s* tmp_node = NULL;

    if (*queue_head == NULL) // empty
    {
        queue_size = 0;
        return NULL;
    }

    if (*queue_head == *queue_tail) // last node
        *queue_tail = (*queue_tail)->next;

    tmp_node = *queue_head;
    *queue_head = (*queue_head)->next;

    queue_size--;

    return tmp_node;
}

float
get_center_dist( cv::Rect center, cv::Point vp )
{
    float min_x = center.x;
    float max_x = center.x + center.width;


    float px = (float)vp.x;

    if( vp.x == 0 && vp.y == 0 )
        return 0;

    float dist = 0;

    if ( px >= max_x ) // the vp is in the right of center point
    {
        dist = px - max_x;
    }

    if( px <= min_x ) // the vp is in the left of center point
    {
        dist = px - min_x;
    }

    return dist;
}


void help()
{
    cout << "\nUsage: "
         << "\n\t --kp <set Kp coef ex: 0.01>"
         << "\n\t --ki <set Ki coef ex: 0.01>"
         << "\n\t --kd <set Kd coef ex: 0.01>"
         << "\n\t --speed <set motor speed ex: 100 (rpm)>"
         << "\n\t --save save the proceed video"
         << "\n\t --help print this message"<< endl;
}

void
get_argv( int argc,
          char** argv,
          _parameter_t &p)
{
    for (int i = 1; i < argc; ++i)
    {
        if (string(argv[i]) == "--kp")
            p.kp = atof( argv[++i] );
        else if (string(argv[i]) == "--ki")
        {
            p.ki = atof( argv[++i] );
        }
        else if (string(argv[i]) == "--kd")
        {
            p.kd = atof( argv[++i] );
        }
        else if (string(argv[i]) == "--speed")
        {
            p.motor_rpm = atoi( argv[++i] );
        }
        else if (string(argv[i]) == "--save")
        {
            p.is_save = true;
        }
        else if (string(argv[i]) == "--help")
        {
            help();
        }
        else
        {
            cout << "Unknown key: " << argv[i] << endl;
        }
    }
}


void
pid_update(PID_t* pid, double curr_error, double dt)
{
    double diff;
    double p_term;
    double i_term;
    double d_term;

    // integration with windup guarding
    pid->int_error += (curr_error * dt);
    if (pid->int_error < -(pid->windup_guard))
        pid->int_error = -(pid->windup_guard);
    else if (pid->int_error > pid->windup_guard)
        pid->int_error = pid->windup_guard;

    // differentiation
    diff = ((curr_error - pid->prev_error) / dt);

    // scaling
    p_term = (pid->proportional_gain * curr_error);
    i_term = (pid->integral_gain     * pid->int_error);
    d_term = (pid->derivative_gain   * diff);

    // summation of terms
    pid->control = p_term + i_term + d_term;

    // save current error as previous error for next iteration
    pid->prev_error = curr_error;
}
