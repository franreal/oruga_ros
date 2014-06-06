ROS_PACKAGES = serialcomm oruga_msgs oruga_bringup oruga_teleop

all:
	for pkg in $(ROS_PACKAGES); do \
          rosmake $$pkg; \
	done

clean:
	for pkg in $(ROS_PACKAGES); do \
          $(MAKE) -C $$pkg clean; \
	done

