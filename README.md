# Yale Bulldogs Racing Arduino Program #

This is the code used to run the Arduino microcontroller in the Yale Bulldogs Racing racecar.

Please use concise, descriptive commit messages.

##Setup guide for Git newbies##

Git is a version control system which automatically keeps track of all changes to the code, and makes them easily visible on Github.  This enables easy collaboration and makes it easy to revert changes if anything gets messed up.

First, create a Github account.  **Email me (geoffrey.litt `at` yale.edu) your username, so I can add you as as collaborator on this repository.**  Otherwise you won't be able to contribute.

Then download Git and set it up:

[Windows help link](http://help.github.com/win-set-up-git/)
[OS X help link](http://help.github.com/mac-set-up-git/)

Then, create a folder you'll use to contain the code.
Navigate to that empty folder and type the following commands:

`git init`  
`git remote add origin git@github.com:geoffreylitt/Yale-Formula-Hybrid.git`  
`git pull origin master`

Now you have the code on your computer! Now how do you modify code and submit it to the central repository?

Basically, the workflow starts with pulling the latest version of the code: `git pull origin master`. Then you make a bunch of changes, and when you're satisfied with them, you add all the files you want to be changed: `git add file1 file2`, or `git add .` to add all files in the folder.  (We'll probably only be dealing with a single file so this is not a big deal). Then you commit the changes with a message attached describing the changes: `git commit -m "added new feature"` At this point the changes are only saved on your local machine, not on the central server. At some regular intervals (maybe at the end of the day, after you've made several commits) you can "push" your changes to the central repository, and then everyone will see them: `git push origin master`

You can also create "branches", which you can imagine as separate forks in the path of development, that make it easier to work in parallel on different sets of changes.

Check out [this link](http://progit.org/book/ch2-0.html) to learn more about branches and other git features.  What I just described is just the tip of the iceberg.