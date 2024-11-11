from glob import glob
import os

package_name = "mypkg"

# (os.path.join('share', package_name), glob("launch/*.launch.py")),
# (os.path.join('share', package_name,  "config"), glob("config/*.yaml")),

# print([os.path.join('share', package_name), glob("launch/*.launch.py")])
l = glob("../launch/*.launch.py")
print(l)
