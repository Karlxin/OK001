1.Target:	
	
2.realizing:	

3.problem:	1.accelerometer is not good for calculate climb rate
		2.the record when baro>5 does not work

4.reason:	

5.todo:		1.altitude hold
		
		
6.phenomenon:	1.remove kalman filter for baro altitude,it works worse.
		2.it works.haha.but not perfect.video:20170411_112132.mp4
		3.hover test 1,video:20170411_113942.mp4
		4.adding 0.06 for hover climb rate P.baro recording for hover throttle are affected by propeller.video:20170411_114952.mp4,20170411_115510.mp4
		5.let the 1400 and 1600 to be coarse estimating,this time it does not record.video:20170411_121034.mp4
		6.down the baro trigger to 6,add filter for angle,highten delay time to 9.04ms for MS5611 conversion make sure of the sanity.it now can hold in roughly 20cm.video:20170411_125704.mp4 
		

7.solutions: 

8.exception:

postscript:    