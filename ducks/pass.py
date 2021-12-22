from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])

        print("YELLOW COLOR IN HSV: ", cv2.cvtColor(np.uint8([[[0, 255, 0]]]), cv2.COLOR_RGB2HSV))

        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])

            img_h, img_w, _ = img.shape
            img_hsv = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_BGR2HSV)

            # Hue is in range from 0 to 179
            mask = cv2.inRange(img_hsv, (85, 100, 150), (95, 255, 255))
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                x, y, rect_w, rect_h = cv2.boundingRect(contours[0])
                maxS = rect_w*rect_h

                for i, contour in enumerate(contours[1:]):
                    _x, _y, _rect_w, _rect_h = cv2.boundingRect(contour)
                    duck_coverage_on_image = rect_h*rect_w/float(img_h*img_w)
                    currS = _rect_w*_rect_h
                    if maxS < currS:
                        maxS = currS
                        x, y, rect_w, rect_h = _x, _y, _rect_w, _rect_h
                    # print("I: ", i)
                duck_coverage_on_image = rect_h*rect_w/float(img_h*img_w)
                print("Max continuous coverage: ", duck_coverage_on_image)
                print("Img: ", img_h, img_w)
                print("Rect: ", x, y, rect_w, rect_h)

                # https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
                # Car length is 0.18 units
                # Assuming that with distance of ~0.2 we see duck height as rougly 170 pixels and actual height is roughly 0.1 units, F ~ 0.2 * 170 / 0.1 = 340
                car_length = 0.18
                F = 340 * 5/6 # Multiply on some correction (empirical) to be closer to the reality

                d = F * 0.1 / rect_h
                print("Distance based on height of found coverage: ", d)
                if d < car_length:
                    condition = False
                    env.step([0, 90])
                    env.render()
                    for i in range(1, 8):
                        env.step([1, 0])
                        env.render()
                    env.step([0, -90])
                    env.render()
                    for i in range(1, 8):
                        env.step([1, 0])
                        env.render()
                    env.step([0, -90])
                    env.render()
                    for i in range(1, 8):
                        env.step([1, 0])
                        env.render()
                    env.step([0, 90])
                    env.render()
                    for i in range(1, 100):
                        env.step([1, 0])
                        env.render()
            else:
                print("Couldn't find contours, movin forward")

            env.render()