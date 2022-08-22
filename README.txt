Welcome to graphs and mazes!

Inside please find graphmaze.py and graphmaze.ipynd
both running Python 3.8. The code and comments in both
will be mostly the same however the Notebook provides
a sandbox environment for personal testing of features
while the more traditional program is meant to operate
using carefully guided user inputs.

This program generate either takes as input one of 101
mazes on 101x101 grids or allows for the user to
nondeterministically populate their own graph of 101x101
along with randomized starting and goal locations.

Moreover, included are 9 different algorithms currently
allowed with the vast majority being different metric
space formulations of the A* algorithm. Initially the
intent was to develop a space where speed and efficiency
of different metrics could be measured, but it involved
to include Bidirectional Search as well. There is room
for further tinkering as well both in the scale and scope
of the program. For instance it is possible to change
the initial starting conditions set as the cost for
traversing laterally is one cheaper than vertically. It
would also be possible, with virtually no extra effort,
to allow for the random generation process to intigrate
graphs of different sizes. As a sandbox concept however
this doesn't work as well when considering further
variables and constraints that must be imposed elsewhere.

To that end this program still represents a nice wait
to get the feel for different metrics. Feel free to try
it and look at the colorful maze solutions that pop out
on the other end!