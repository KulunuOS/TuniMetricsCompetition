# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 16:41:38 2022

@author: monakhov
"""


import detectron2
import cv2
import os
#os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"
import json_submitter

from detectron2.utils.logger import setup_logger
setup_logger()
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
import numpy as np
import sys
def main(argv):
    submitter = json_submitter.JSON_SUB()
    for imname in argv:
        print(imname)
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-Detection/faster_rcnn_R_50_FPN_3x.yaml"))
    cfg.DATASETS.TRAIN = ()
    cfg.DATASETS.TEST = ()
    cfg.MODEL.DEVICE='cpu'
    cfg.DATALOADER.NUM_WORKERS = 0
    # Let training initialize from model zoo
    cfg.SOLVER.IMS_PER_BATCH = 8
    cfg.SOLVER.BASE_LR = 0.005
    cfg.SOLVER.MAX_ITER = 7000
    cfg.SOLVER.CHECKPOINT_PERIOD = cfg.SOLVER.MAX_ITER//4
    cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 128   # faster, enough for this dataset (default: 512)
    cfg.TEST.EVAL_PERIOD = cfg.SOLVER.MAX_ITER//4
    cfg.OUTPUT_DIR = r"output"
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 3
    ##

    cfg.MODEL.WEIGHTS = 'alpha_noise_physics_half.pth'
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.4   # set a testing threshold
    predictor = DefaultPredictor(cfg)
    
    for imname in argv:
        im = cv2.imread(imname)
        outputs = predictor(im)
        boxes = outputs['instances'].pred_boxes.tensor.cpu().numpy()
        boxes = np.int32(boxes)
        for inst_ind,box in enumerate(boxes):
            cv2.rectangle(im,(box[0],box[1]),
                      (box[2],box[3]),(255,0,0),3)
            im = cv2.putText(im, str(outputs['instances'].pred_classes.cpu().numpy()[inst_ind]),
                              (box[0],box[1]+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0),
                            2, cv2.LINE_AA)
            newimname = os.path.basename(imname)
            newimname = newimname.split('.')[0]        
            cv2.imwrite(newimname+'_res.png',im)
            submitter.CollectRecord(outputs['instances'], inst_ind, imname)

    submitter.DumpJSON()
    
if __name__ == "__main__":
    main(sys.argv[1:])


