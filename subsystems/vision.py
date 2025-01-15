#pip install limelightlib-python
#https://docs.limelightvision.io/docs/docs-limelight/apis/limelightlib-python
#code is taken from the documentation, modified for robotpy, and comments were added for clarity

import limelight
import limelightresults
import json
import time
from math import tan, radians


class robot_camera():
    def __init__(self):
        self.camera_height = 40
        self.camera_angle = 21

        discovered_limelights = limelight.discover_limelights()
        #discover_limelights returns a list of all limelights found on network
        print(discovered_limelights)

        #limelight initiation ect will not appen unless a limelight is found
        if discovered_limelights:
            self.camera_found = True
            limelight_address = discovered_limelights[0]
            #this is selecting the first limelight from the list
            self.ll = limelight.Limelight(limelight_address)
            #this is the main limelight class that allows you to talk to the limelight
            #ll functions use HTTP GET to request data from the limelight
            results = self.ll.get_results()
            status = self.ll.get_status()
            print("-----")
            print("targeting results:", results)
            print("-----")
            print("status:", status)
            print("-----")
            print("temp:", self.ll.get_temp())
            print("-----")
            print("name:", self.ll.get_name())
            print("-----")
            print("fps:", self.ll.get_fps())
            print("-----")
            print("hwreport:", self.ll.hw_report())
            #this prints out all of the basic hardware info on initiation

            self.ll.enable_websocket()
        
            # print the current pipeline settings
            print(self.ll.get_pipeline_atindex(0))

            # update the current pipeline and flush to disk
            pipeline_update = {
            'area_max': 98.7,
            'area_min': 1.98778
            }
            self.ll.update_pipeline(json.dumps(pipeline_update),flush=1)

            print(self.ll.get_pipeline_atindex(0))

            # # switch to pipeline 1
            # self.ll.pipeline_switch(1)

            # update custom user data
            self.ll.update_python_inputs([4.2,0.1,9.87])
        
        #camerafound is used later to avoid calling a camera if none are present
        else:
            self.camera_found = False
    
    def find_distance(self,camera_height, tag_height, camrera_angle, ty):
        height = tag_height-camera_height
        tag_angle = camrera_angle-ty

        distance = height/(tan(radians(tag_angle)))

        return(distance)

    def run(self):
        tag_height = 60.75

        if self.camera_found == True:
            result = self.ll.get_latest_results()
            parsed_result = limelightresults.parse_results(result)
            if parsed_result is not None:
                if parsed_result.validity > 0:
                    distance = self.find_distance(self.camera_height, tag_height, self.camera_angle, result['ty'])
                    print(distance)
                else:
                    print('no target found')
                # print("valid targets: ", parsed_result.validity, ", pipelineIndex: ", parsed_result.pipeline_id,", Targeting Latency: ", parsed_result.targeting_latency)
                # for tag in parsed_result.fiducialResults:
                #    print(tag.robot_pose_target_space, tag.fiducial_id)
            time.sleep(0)  # Set this to 0 for max fps
            #this is for the interval between getting results, set to 0 as to not slow down teleop function
    def end(self):
        if self.camera_found == True:
            self.ll.disable_websocket()
            #this is shutting down the limelight