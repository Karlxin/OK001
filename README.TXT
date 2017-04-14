1.Target:	
	
2.realizing:	

3.problem:	1.accelerometer is not good for calculate climb rate
		2.the record when baro>5 does not work
		3.it roll wobble by cycle.

4.reason:	1.51 percent by the noise,we reduce the roll wobble by decreasing the P of omega many times 

5.todo:		1.altitude hold
		
		
6.phenomenon:	1.remove kalman filter for baro altitude,it works worse.
		2.it works.haha.but not perfect.video:20170411_112132.mp4
		3.hover test 1,video:20170411_113942.mp4
		4.adding 0.06 for hover climb rate P.baro recording for hover throttle are affected by propeller.video:20170411_114952.mp4,20170411_115510.mp4
		5.let the 1400 and 1600 to be coarse estimating,this time it does not record.video:20170411_121034.mp4
		6.reduce the baro trigger to 6,add filter for angle,highten delay time to 9.04ms for MS5611 conversion make sure of the sanity.it now can hold in roughly 20cm.video:20170411_125704.mp4 
		7.reduce the omega by 0.05.not better than previous one.video:20170411_131757.mp4
		8.theta_P down to 2.5,omega_P down to 0.3,omega_yaw_P down to 0.6.video:it tilt to the rear,video:20170411_183024.mp4
		8.1change oemga_P up to 0.45,theta_P up to 2.8.it tilt to the rear.video:20170411_183910.mp4
		8.2.I haven't added the throttle cybernation after test the motor vibration effect to gyros,video:20170411_184856.mp4
		9.adding cybernation and down the P to 2.5,omega to 0.3,yaw to 0.6.the machine is too flexible that looks like less stable.video:20170411_185747.mp4
		10.Theta_p to the 2.8,omega_p to the 0.45,it is also too flexible.video:20170411_190545.mp4
		11.P to 2.5,D to 0.45 0.45 0.6.video:20170411_194742.mp4
		12.P to 2.3,D to 0.46 0.46 0.6.safety.video:20170414_121015.mp4

7.solutions: 

8.exception:

postscript:    