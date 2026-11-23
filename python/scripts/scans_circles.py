from deltabot import DeltaBot
from deltabot import scan
from deltabot import camera
import numpy as np

with DeltaBot() as bot:          # "sim" to try it without hardware
    cam = camera.Camera()
    sc = scan.Scanner(bot, cam, data_dir="scans/data")
    cam.remote.enable_hough(min_radius=4, max_radius=7, param2=22, min_dist=50)
    # bot.calibrate_home()
    # # bot.set_speed(600)           # steps/s
    # bot.move_xy(0.5, 0.25)
    # # bot.nudge_xy(dx=-0.1)
    # # bot.move_polar(r=1.0, theta_deg=30)

    sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")

    #initial scans - circles centred on 0,0 at varying radii
    for t_steps in [62]:
        for radius in [1.2, 1, 0.8, 0.6, 0.3]:
            t = np.linspace(0, 2 * np.pi, t_steps)
            sc.run(list(zip(radius * np.cos(t), radius * np.sin(t))), scan.Circles(), scan.TrackedCircle(roi_width=280), name= "circles")

    #return to centre
    sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")

    #9 medium scans 
    for y_offset in [0.8, 0, -0.8]: 
        for x_offset in [0.8, 0, -0.8]: 
            for t_steps in [62]:
                for radius in [(0.4)]:
                    t = np.linspace(0, 2 * np.pi, t_steps)
                    sc.run(list(zip(radius * np.cos(t) + x_offset, radius * np.sin(t) + y_offset)), scan.Circles(), scan.TrackedCircle(roi_width=280), name= "circles")
    
    #return to centre
    sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")
    
    #repeat initial scans - circles centred on 0,0 at varying radii
    for t_steps in [62]:
        for radius in [1.2, 1, 0.8, 0.6, 0.3]:
            t = np.linspace(0, 2 * np.pi, t_steps)
            sc.run(list(zip(radius * np.cos(t), radius * np.sin(t))), scan.Circles(), scan.TrackedCircle(roi_width=280), name= "circles")

    #return to centre
    sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")

    #1 more big scan with more points
    t = np.linspace(0, 2 * np.pi, 124)
    sc.run(list(zip(1.2 * np.cos(t), 1.2 * np.sin(t))), scan.Circles(), scan.TrackedCircle(roi_width=280), name= "circles")

    #return to centre
    sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")

    bot.home
    print(bot.pose(), bot.z(), bot.strain())
