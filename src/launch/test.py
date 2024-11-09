
import os
from glob import glob


# (os.path.join('share', 'mypkg'), glob("launch/*.launch.py")),

# print(os.path.join('share', 'mypkg'), glob("launch/*.launch.py"))

# print(glob("*.launch.py"))
# print(os.path.join('share', 'mypkg'), glob("launch/*.launch.py"))


# print(os.path.join('share', 'mypkg'), glob("*.launch.py"))

# print(os.path.join('share', "mypkg", 'launch'), glob(os.path.join(os.getcwd(), 'launch', '*.launch.py')))

print(glob(os.path.join(os.getcwd(), '*.launch.py')))
