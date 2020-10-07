## ROS Network Setup

ROS is a distributed computing environment. This allows running compute expensive tasks such as visualization or path planning for navigation
on machines with more performance and sending goals to robots operating on less performant hardware like DiffBot with its Raspberry Pi 4 B.

For detailed instructions see [ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup) and the 
[ROS Environment Variables](http://wiki.ros.org/ROS/EnvironmentVariables).

The setup between the work machine that handles compute heavy tasks and DiffBot is as follows:

TODO image


On DiffBot we configure the `ROS_MASTER_URI` to be the IP address of the work machine.

```console
export ROS_MASTER_URI=http://192.168.0.9:11311/
```

To test run the master on the work machine using the `roscore` command.
Then run the the `listener` from the `roscpp_tutorials` package in another terminal:

```console
fjp@workmachine:~/git/diffbot/ros$ rosrun roscpp_tutorials listener

```

Then switch to a terminal on DiffBot's Raspberry Pi and run the `talker` from `roscpp_tutorials`:

```console
fjp@diffbot:~/git/diffbot/ros$ rosrun roscpp_tutorials talker 
[ INFO] [1602018325.633133449]: hello world 0
[ INFO] [1602018325.733137152]: hello world 1
[ INFO] [1602018325.833112540]: hello world 2
[ INFO] [1602018325.933114483]: hello world 3
[ INFO] [1602018326.033114093]: hello world 4
[ INFO] [1602018326.133112684]: hello world 5
[ INFO] [1602018326.233112183]: hello world 6
[ INFO] [1602018326.333113126]: hello world 7
[ INFO] [1602018326.433113680]: hello world 8
[ INFO] [1602018326.533113031]: hello world 9
[ INFO] [1602018326.633110140]: hello world 10
[ INFO] [1602018326.733108954]: hello world 11
[ INFO] [1602018326.833113267]: hello world 12
[ INFO] [1602018326.933164505]: hello world 13
[ INFO] [1602018327.033119135]: hello world 14
[ INFO] [1602018327.133113559]: hello world 15
[ INFO] [1602018327.233111003]: hello world 16
[ INFO] [1602018327.333110705]: hello world 17
[ INFO] [1602018327.433126425]: hello world 18
[ INFO] [1602018327.533111498]: hello world 19
[ INFO] [1602018327.633107978]: hello world 20
[ INFO] [1602018327.733110736]: hello world 21
[ INFO] [1602018327.833107605]: hello world 22
[ INFO] [1602018327.933111659]: hello world 23
[ INFO] [1602018328.033108065]: hello world 24
[ INFO] [1602018328.133110379]: hello world 25
[ INFO] [1602018328.233150191]: hello world 26
[ INFO] [1602018328.333135986]: hello world 27
[ INFO] [1602018328.433153558]: hello world 28
[ INFO] [1602018328.533154557]: hello world 29
[ INFO] [1602018328.633151667]: hello world 30
[ INFO] [1602018328.733128777]: hello world 31
[ INFO] [1602018328.833170108]: hello world 32
[ INFO] [1602018328.933172402]: hello world 33
```

Looking back at the terminal on the work machine you should see the output:

```console
fjp.github.io git:(master) rosrun roscpp_tutorials listener
[ INFO] [1602018328.330070016]: I heard: [hello world 27]
[ INFO] [1602018328.430244670]: I heard: [hello world 28]
[ INFO] [1602018328.530173113]: I heard: [hello world 29]
[ INFO] [1602018328.630251690]: I heard: [hello world 30]
[ INFO] [1602018328.730334064]: I heard: [hello world 31]
[ INFO] [1602018328.830346566]: I heard: [hello world 32]
[ INFO] [1602018328.930009032]: I heard: [hello world 33]
```

Note that it can take some time receiving the messages from DiffBot on the work machine, which we can see from the time stamps in the outputs above.
