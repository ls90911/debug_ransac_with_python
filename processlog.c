
#include <stdio.h>

#include "dronerace/filter.h"
#include "dronerace/ransac_run.h"
#include "dronerace/control.h"



int main(void)
{
	float phi = 0, theta = 0, psi = 0;
	float dx = 0, dy = 0, dz = 0;
	int cnt = 1;
	int cnt_ref = 1;
	float dt = 0.01f;

	FILE* fp = fopen("log.txt","r+");
	FILE* out = fopen("result.txt","w");

	if (fp == NULL)
	{
		fprintf(stderr,"Unable to open log.txt\n");
		return -1;
	}

	if (out == NULL)
	{
		fprintf(stderr,"Unable to open result.txt\n");
		return -1;
	}

	filter_reset();
	control_reset();

	while(1)
	{
		int ret = fscanf(fp,"%f,%f,%f,%d,%f,%f,%f\n",&phi,&theta,&psi,&cnt,&dx,&dy,&dz);
		// printf("scanf = %d\n",ret);

		if (ret == EOF)
			break;



		///////////////////////////////////////////////////////////
		// Call filter
		filter_predict(phi,theta,psi,dt);
		control_run(dt);

		// Update vision
		if (cnt > cnt_ref)
		{
			cnt_ref = cnt;
			dr_vision.cnt = cnt;
			dr_vision.dx = dx;
			dr_vision.dy = dy;
			dr_vision.dz = dz;
			filter_correct();
		}
		///////////////////////////////////////////////////////////


		fprintf(out,"%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%f,%f\n",
		  dr_state.time,
		  dr_state.x, // + dr_ransac.corr_x, 
			dr_state.y, // + dr_ransac.corr_y, 
		  dr_state.x + dr_ransac.corr_x, 
			dr_state.y + dr_ransac.corr_y, 
			dr_state.vx, 
			dr_state.vy,
      dr_ransac.buf_size,
      dr_vision.cnt,
	  dr_ransac.ransac_cnt,
			log_mx,
			log_my
			);
	}

	if(feof(fp))  
	{            
		puts("EOF");     
	}  
	else  
	{  
		puts("CAN NOT READ");  
	}

	fclose(fp);
	fclose(out);
 	return 0;
}
