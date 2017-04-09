#include<pthread.h>
#include<stdio.h>
#include<semaphore.h>
#include"dlab.h"

#define pi 3.1415926535

#define MAXS 5000


pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
sem_t data_avail;
pthread_attr_t attr;
//pthread_attr_init(&attr);
//pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

pthread_t ControlThread;
pthread_t changethread;
pthread_t scanthread;
pthread_t c_ithread;

float theta[MAXS];
float ref[MAXS];
float theta_store[MAXS];

float Ti = 0.02;
float Td = 0.005;
float Tt = 0.01; //for saturation block
float N = 20; // for derivative term
float kp = 30.0; 
float runt = 3; //run time
float Fs = 200.0; // sampling frequency
int motor_no = 5; // lab stn
int k; // sample number
int input; // square or step input choice storage
float mag; // magnitude of input
float duty = 50; // duty cycle for square input
float sq_freq = 5; // frequency for square wave input
char selection;
 // main menu selection storage
int no_of_samples;   // number of samples to be displayed in total

char stop;

char ch; // selecting which parameter to change
int j; // this is 3 sec worth of samples
int i;

float satblk(float out)
{
float threshold = 1.4; // for max voltage

float threshold2 = -1.4;
if (out > threshold)
{
out = threshold; // making value of v less than max allowed I am not sure about this function
}

if (out < threshold2)
{
out = threshold2;
}

return(out);

}

void *scan(void*arg)
{

  printf("Give value");
while(1)
{
scanf("%c", &ch);
 if (ch == 's')
   break;
}
}

void*c_i(void*arg) // make for ti, td, kp, N, Fs
{
if (ch == 's')
  printf("stopped ");
}


void *Control(void *arg){
	k=0;
	no_of_samples = (int)(runt*Fs);
	float ek_old = 0;
	float Dk_old = 0;
	float Ak_old = 0;
	float Ik_old = 0;
	float T = 1.0/Fs;
     
	/*	  scanf("%c", ch);
	  if(ch == 's')
	    break;
	  if(ch == 'i')
	    { printf("input new Ti: ");
	      scanf("%f", &Ti);
	    }
	  if(ch == 'n')
	    {    k=0;
	    }
	    */
	  while (k<no_of_samples) {
	    
		sem_wait(&data_avail);
		float motor_position = EtoR(ReadEncoder());
		float ek = ref[k] - motor_position;
		float Pk = kp*ek;  // proportional term based on discrete time
		float Ik = Ik_old + (kp/Ti)*ek_old*T + ((1/Tt)*(Ak_old)*T); // Integral based on discrete time
		float Dk = (Td/(N*T+Td)*Dk_old + (kp*Td*N)/(N*T + Td)*(ek - ek_old));
		
		
		//Dk = (Td/(N*T +Td))*(Dk_old) + ((kp*Td*N)/(N*T + Td))*(ek - ek_old)); // derivative term based on discrete time
		float vk = Pk + Ik + Dk; //total ouput
		float uk = satblk(vk);
		float Ak = uk - vk;
		//float a = uk - v;
		//float Ikv = Ik+((a/Tt));
	        //v = Pk + Ikv + Dk;
		//float u = satblk(v);h
		//Ik_prev;
		//Ak_prev;
		DtoA(VtoD(uk));
		theta[k] = motor_position;
		printf("%f  ", theta[k]);
		if(j<no_of_samples)
		{
		  //theta[k]=theta_store[j];
		j++;
			if(j==no_of_samples)
				{
					j=0;
				}
		}
		k++;
		while (k == no_of_samples)
		  {
		    printf("\nPress 'n' to not change paramaters and run the next three secs:\n ");
		    printf("Press 's' to stop the simulation: \n");
		    printf("Press 'i' to change Ti: \n");
		    printf("Press 'd' to change Td: \n");
		    printf("Press 'N' to change N: \n");
		    printf("Press 'k' to change kp: \n");
		    printf("Press 'f' to change Fs: \n");

		    scanf("%c", &ch);
		    if(ch == 's')
		      break;
		    if(ch == 'i')
		      { printf("input new Ti: ");
			scanf("%f", &Ti);
		      }
		    if(ch == 'd')
                      { printf("input new Td: ");
                        scanf("%f", &Td);
                      }
		    if(ch == 'N')
                      { printf("input new N: ");
                        scanf("%f", &N);
                      }
		    if(ch == 'k')
                      { printf("input new kp: ");
                        scanf("%f", &kp);
                      }
		    if(ch == 'f')
                      { printf("input new Fs: ");
                        scanf("%f", &Fs);
                      }
		    if(ch == 'n')
		      {    k=0;
		      }
		    i++;
		  }
		ek_old = ek; // for e((k-1)T) term, I am not sure what to use for T though
	        Dk_old = Dk; // for D((k-1)T) term, I am not sure what to use for T thoug
		Ak_old = Ak;
		Ik_old = Ik;

		  }

	  } 


int main() {
	for(k = 0; k<MAXS; k++) {
		ref[k] = 0;
	}

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	while(1) {
		printf("Enter\n"); 
		printf("'r' to run the control algorithm: \n");
		printf("'p' to Change value of Kp. \n");
		printf("'f' to Change value of sample freq: \n");
		printf("'t' to Change value of run time: \n");
		printf("'u' to Change the type of inputs (Step or Square): \n");
		printf("'g' to Plot results on screen: \n");
		printf("'h' to Save the Plot results in Postscript \n");
	        printf("'i' to change Ti: \n");
	        printf("'d' to change Td: \n");
	        printf("'n' to change value of N: \n");
		printf("'q' to exit: \n");
       
		scanf(" %c", &selection);

		switch (selection) {
			case 'r':
			sem_init(&data_avail, 0, 0);
			Initialize(DLAB_SIMULATE, Fs, motor_no);
			pthread_create(&ControlThread, NULL, &Control, NULL);
			pthread_join(ControlThread, NULL);
			//	pthread_create(&scanthread, &attr, &scan, NULL);
			//pthread_join(scanthread, NULL);
			//	pthread_create(&c_ithread, &attr, &c_i, NULL);
			//pthread_join(c_ithread, NULL);
			
			
			Terminate();
			sem_destroy(&data_avail);
			break;

			case 'u':
			//prompt user for type of input: Step or Square
			printf("Enter type of input (Enter 1 for step, 2 for square): ");
			scanf(" %d", &input);
			if (input == 1) {
				printf("Enter magnitude (*pi/180 radians) of step: ");//prompt for magnitude of the step
				scanf("%f", &mag);
				mag = mag*pi/180;
				for(k = 0; k<MAXS; k++) {
						ref[k] = mag; 
				} //set up the step reference input {ref[k]}
			}

			if(input == 2) {
				printf("Enter Magnitude(*pi/180 radians), Frequency and duty cycle(in integer) respectively: ");
				scanf(" %f %d %d", &mag, &sq_freq, &duty); // ex. 50*pi/180.0, 0.5, 50
				no_of_samples = (int)(Fs*runt);
				mag = mag*pi/180;

				Square (ref, no_of_samples, Fs, mag, sq_freq, duty);//set up {ref[k]} using DLaB function Square()
			}
			break;

			case 'p':
			printf("Enter new value of kp: "); //Prompt user for new value of Kp
			scanf(" %f", &kp);
			break;
		case 'f':
                  printf("Enter new value of Fs: ");
                  scanf(" %f", &Fs);
                  break;
		case 't':
                  printf("Enter new value of run time: ");
                  scanf(" %f", &runt);
                  break;
			case 'g':
			no_of_samples = (int)(Fs*runt);
			plot (theta, ref, Fs, no_of_samples, SCREEN, "Response", "Last 3 seconds", "theta (radians)");
			case 'h':
			no_of_samples = (int)(Fs*runt);
			plot (theta, ref, Fs, no_of_samples, PS, "Response", "Last 3 seconds", "theta (radians)");
			break;
		case 'i':
		  printf("Enter new value of i: "); 
		  scanf(" %f", &Ti);
		  break;
		case 'd':
                  printf("Enter new value of d: ");
                  scanf(" %f", &Td);
                  break;
		case 'n':
                  printf("Enter new value of N: ");
                  scanf(" %f", &N);
                  break;
			case 'q':
			printf("We are done!");
			exit(0);

			case 's':
			Terminate();
			sem_destroy(&data_avail);
			break;

			default:
			printf(" Invalid input "); //Invalid selection, print out error message;
			break;

			// I am not sure whether to do the code below this or above it The one above uses the Gain function and the one below doesn't
			/*ek = rek[k] - motor_position;
			Pk  = Kp*ek;
			Ik = (Ik_prev) + (Kp/Ti)*((ek_prev)*T) + ((1/Tt)*(Ak_prev)*T;
			Dk = ((Td/(N*T) + Td)) * (Dk_orev)) + (((Kp*Td*N)/(N*T) + Tf)*(ek - (ek_prev);
			vk = Pk + Ik+ Dk;
			uk = (sat_block(vk));
			Ak = uk - vk;*/
		}

	}
}
