from deltabot import DeltaBot
from deltabot import scan
from deltabot import camera
import csv
import numpy as np
import pandas as pd
import tensorflow as tf
from joblib import dump, load
from sklearn.preprocessing import StandardScaler

from processing import process_the_data, series_to_supervised


"""Basic script which 
    1. drives the bot window_size steps using the normal DeltabotController Algorithm
    2. asks the user for input for the next desired position of the bot (Note: there is no error handling for non-float inputs.)
    3. uses a neural network (saved/imported .keras file) to predict the motor steps needed to get to that location
    4. drives the bot to the new neural network steps. 
    5. repeates steps 2. to 4. indefinitely (until the user ctrl-c)
    """

with DeltaBot() as bot:  # port="sim" to try it without hardware
    cam = camera.Camera()
    # sc = scan.Scanner(bot, cam, data_dir="scans/data")
    hough = cam.remote.enable_hough(min_radius=5, max_radius=9, param2=22, min_dist=50)

    window_size = 10

    # first window_size steps to get history
    fh = open("data.csv", "w", newline="")
    writer = None
    with fh:
        for y in [0]:
            for x in [-x / 10.0 for x in range(window_size+ 1)]:
                pose = bot.move_xy(x, y, scan.Circles()) #, scan.TrackedCircle())

                frame = cam.acquire(hough=hough)

                row = {"x": pose.x, "y": pose.y}
                row["frame_id"] = frame.frame_id
                steps = bot.positions()
                row.update({"s0": steps[0], "s1": steps[1], "s2": steps[2]})

                row.update(dict(scan.Circles().read(frame, None)))
                if writer is None:
                    writer = csv.DictWriter(fh, fieldnames=list(row), restval="")
                    writer.writeheader()
                    initial_pos = row
                writer.writerow(row)
                fh.flush()


        while(True): #infinitely predict next steps 

            #convert measured cx and cy values into mm
            px_per_mm = 11.85
            data = process_the_data("", "", 0.1 / px_per_mm, 0.1 / px_per_mm, initial_pos["cx"], initial_pos["cy"])
            data_processed = data[["cx_mm", "cy_mm", "s0", "s1", "s2"]]

            #convert measured values into a window of history - so neural network knows where it has been 
            series = series_to_supervised(data_processed, window_size-1, 1)

            print("series \n", series)
            
            inputs = series

            #gather from user where bot should go next
            print("current position: \n", data_processed.iloc[-1])
            next_x = input("x:") 
            next_y = input("y:")

            inputs["var1(t+1)"] = float(next_x)
            inputs["var2(t+1)"] = float(next_y)
            inputs["cx_diff"] = inputs["var1(t+1)"] - inputs["var1(t)"]
            inputs["cy_diff"] = inputs["var2(t+1)"] - inputs["var2(t)"]
            inputs = inputs[1:]
            
            print("inputs: \n", inputs)

            inputs.to_csv(f"window_{window_size}_test.csv")

            #scale data
            bot_test = inputs
            bot_test.iloc[[-1]]

            scalerX = load(f'scalerX_{window_size}.bin') #from training data
            scalerY = load(f'scalerY_{window_size}.bin') #from training data

            bot_test_scaled = scalerX.transform(bot_test.iloc[[-1]])

         
            #use model to predict steps
            model = tf.keras.models.load_model(f'window_{window_size}_dropout.keras') #need to work out how to get tf installed
            bot_test_pred = model.predict(bot_test_scaled)

            #inverse transform to conver to motor steps
            bot_test_pred_inverse = scalerY.inverse_transform(bot_test_pred)
            bot_test_pred_inverse = np.append(bot_test_pred_inverse, -(bot_test_pred_inverse[:,0] + bot_test_pred_inverse[:,1])) #calculate s2
            print(bot_test_pred_inverse.astype("int32")) 
             
            #move bot to predicted steps value
            bot.move_steps(bot_test_pred_inverse[0], bot_test_pred_inverse[1], bot_test_pred_inverse[2]) 
            
            #measure cx, cy
            frame_new = cam.acquire(hough=hough)
            row["frame_id"] = frame_new.frame_id
            steps = bot.positions()
            row.update({"s0": steps[0], "s1": steps[1], "s2": steps[2]})
            row.update(dict(scan.Circles().read(frame_new, None)))

            #process cx, cy
            writer.writerow(row)
            fh.flush()
            new_data = process_the_data("", "", 0.1 / px_per_mm, 0.1 / px_per_mm, initial_pos["cx"], initial_pos["cy"])

            #compare!
            print("desired position: ", next_x, next_y)
            print("new position: \n", new_data[["cx_mm", "cy_mm", "s0", "s1", "s2"]]) #.iloc[-1]


    bot.home
    print(bot.pose(), bot.z(), bot.strain())

