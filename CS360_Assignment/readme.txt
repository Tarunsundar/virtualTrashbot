Instructions on running my solution:
1. Open up your terminal(operating system must be linux)
2. run roscore(enter the text and hit enter) in a termninal tab
3. Open up a new tab and execute roslaunch CS360_Assignment image_proc.launch
4. Open up a new tab and execute rosrun CS360_Assignment pubvel_360_findObjects.py

Bonus
Inorder to change the number of objects in the bin
1.open up the scripts folder in the project's main directory
2.Open up pubvel_360_findObjects.py
3.under line 199 change the value in "numberOfObjThrown == value" to number of objects you want to throw (note: but there must be as many objects as mentioned)
