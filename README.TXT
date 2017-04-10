1.Target:	
	
2.realizing:	

3.problem:	1.accelerometer is not good for calculate climb rate

4.reason:	

5.todo:		1.altitude hold
		
6.phenomenon:	1.we open the altitude hold,to see the noise.it does not meet the requirement video:20170409_123729.
		2.change althold range to 10,it can hold,but it is not perfect,it could go down so how,we need refine it,video:20170409_132647.mp4
		3.we let Scd up to 20,and let des condition range from 0.5 to 1,let us see what will happen now.it crashed somehow video:20170409_155835.mp4
		4.
		4.1change Scd back to 15
		4.2and the condition back to 0.5
		4.3change condition pitch roll to 1
		4.4change kalman baro climb R to 3
		4.5adding throttle bound record accord to accelerometer
		4.6remove the integral condition constraints for climb rate
		4.7adding body frame accz to earth frame accz for integral climb rate
		4.8we use kalman filter for baro climb rate
		5.
		5.1remove the complementation inside the 1 herz loop.
		5.2move the complementation to baro loop.
		5.3use baro climb rate for record the hover throttle up bound.
7.solutions: 

8.exception:

postscript:    