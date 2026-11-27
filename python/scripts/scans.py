from deltabot import DeltaBot
from deltabot import scan
from deltabot import camera

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

    #initial scan - large square - +ve to -ve
    for (x_steps, y_steps) in [(25, 25)]:
        for (x, y) in [(1.2,1.2)]:
            sc.mesh(x, -x, x_steps, y, -y, y_steps,  scan.Circles(), scan.TrackedCircle(roi_width=280), snake=True)
            sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")
    
    #9 smaller scans -ve to +ve
    for y in [-0.8, 0, 0.8]: 
        for x in [-0.8, 0, 0.8]: 
            for steps in [17]: 
                bot.move_xy(x, y)
                sc.run([(x, y)] * 2, scan.Circles(), scan.TrackedCircle(), settle=1.0, name="centre")
                sc.dmesh(-0.4, 0.4, steps, -0.4, 0.4, steps,  scan.Circles(), scan.TrackedCircle(roi_width=280), snake=True) #relative scan 
    
    #return to centre
    sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")
    
    #repeat initial scan but smaller area - hope to stay in the field of view -+ve to -ve
    for (x_steps, y_steps) in [(21, 21)]:
        for (x, y) in [(1,1)]:
            sc.mesh(x, -x, x_steps, y, -y, y_steps,  scan.Circles(), scan.TrackedCircle(roi_width=280), snake=True)
            sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")
    
    #repeat prev scan but -ve to +ve
    for (x_steps, y_steps) in [(21, 21)]:
            for (x, y) in [(1,1)]:
                sc.mesh(-x, x, x_steps, -y, y, y_steps,  scan.Circles(), scan.TrackedCircle(roi_width=280), snake=True)
                sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")

    sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")

    bot.home
    print(bot.pose(), bot.z(), bot.strain())
