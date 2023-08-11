Fix WSL2 by making gazebo use CPU 

```
export LIBGL_ALWAYS_SOFTWARE=1
```

Include setup gazebo in .bashrc
```
stat /usr/share/gazebo/setup.sh &> /dev/null
if [ $? -eq 0 ]; then
    source /usr/share/gazebo/setup.sh
fi
```
