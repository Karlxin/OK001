1.Target:	
	
2.realizing:

3.problem:	1.back to stable position with a little bit wobble 

4.reason:	1.P
		2.noise

5.todo:		1.adjust P
		2.adding frequency
		3.adding alpha P

6.phenomenon:
		1.rearrange schedule,this must be the key,video:20170406_102028.mp4
		2.Nothing changed.video:20170406_131328.mp4
		3.arranged.some oscillation exist.video:20170406_140855.mp4
		4.using kalman,but oscillation bigger,video:20170406_145544.mp4
		5.change back,it works better,but a little oscillation sometimes,video:20170406_151443.mp4
		6.omegaP all added by 0.1.looks like a little better?,video:20170406_172310.mp4
		7.adding 200Hz for gyrometer and accelerometer.no better,video:20170406_174328.mp4
		8.gyro_480,dmp_180,c&u_100,video:20170406_180350.mp4
		9.no big change.video:20170406_205516.mp4
		10.adding alpha circle,video:20170406_210436.mp4
		11.adding sliding window for alpha,crashed,noise is too big,or maybe the omega constraints is too big.video:20170407_083451.mp4
		12.changing back,wobble exists.video:20170407_084544.mp4

7.solutions: 

8.exception:

postscript:    