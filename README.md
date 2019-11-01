# src
git repo for autonomous systems

To do
===

Main function: Bernardo

Observation interpreter: Inês

Odometry interpreter: Luís

*Fake data:* Henrique

How to use this repo
===

To avoid problems with catkin build and so forth

1

Rename your src folder in ros_ws to src-local (or whatever)

2

git clone this repo (src folder) to ros_ws

3

catkin build (everything should build nicely)

4

Copy any packages you've been working on to src (from src-local)
catkin build again

5

to start working on your task, create a new branch

```bash
cd ~/ros_ws/src
git branch <name-of-the-branch>
git checkout <name-of-the-branch>
```

you can now work and try and fail without compromising the branch master

6

When you think everything is ok, commit your changes to your branch
```bash
git add .
git commit -am "commit message, write something nice"
git push origin <name-of-the-branch>
```

7

After a group discussion, a decision on merging the branch is made and the
responsible for the repo will do
```bash
git checkout master
git merge <name-of-the-branch-to-merge>
```

8

If after a while you want to restart your work on the project, don't forget to get the most recent version of the master
```bash
git pull
```



