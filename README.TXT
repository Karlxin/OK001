1.目标:	
	
2.实现：

3.问题：1.channel3_in goes to 20000 or even bigger number sometimes.
	2.actually pwm capturer goes worong,we must fix it.
	3.yaw problem
	4.control problem

4.原因：1.use the simulation could go wrong
	2.definitely the read data frequency

5.尝试：

6.现象: 1.什么都没加，光测试,总觉得没以前好稳哈，操控有点钝钝的.视频:20170405_152521.mp4

	2.改动很大，操控还是没有以前棒,视频:20170405_192120.mp4

	3.xyz_t_3.5,z_o_0.7,y_o_0.5,x_o_0.6,摆来摆去的，哈哈,视频:20170405_205207.mp4

	4.rearrange schedule,this must be the key,vedio:

7.解决: 

8.异常:

PS:    