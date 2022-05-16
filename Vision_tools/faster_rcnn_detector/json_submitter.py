# -*- coding: utf-8 -*-
"""
Created on Fri Feb 11 14:19:02 2022

@author: monakhov
"""
import json
class JSON_SUB:
    
    def __init__(self):
        self.results = []
        return
    
    def CollectRecord(self,instance_output,instance_index,imname):
        tmpres = {}
        box = instance_output.pred_boxes.tensor.cpu().numpy()[instance_index]
        tmpres['image_name'] = imname
        tmpres['category_id'] = int(instance_output.pred_classes.cpu().numpy()[instance_index]+1)
        tmpres['bbox'] = [int(box[0]),int(box[1]),int(box[2]-box[0]),int(box[3]-box[1])]
        tmpres['score'] = float(instance_output.scores.cpu().numpy()[instance_index])
        self.results.append(tmpres)
        
    def DumpJSON(self,filepath='results.json'):
        with open(filepath, 'w') as fout:
            json.dump(self.results, fout)