# Branch of OpenICV

Source code of OpenICV project.

## Index
  - [Tools:](#Tools)
  - [Examples:](#Examples)
  - [Include:](#Include)
  - [Source:](#Source)
  - [Third Party:](#ThirdParty)
  - [evaluation/eval_on_val_for_metrics.py:](#evaluationeval_on_val_for_metricspy)
  - [Visualization](#visualization)
    - [visualization/run_on_seq.py:](#visualizationrun_on_seqpy)
    - [visualization/run_on_thn_seq.py:](#visualizationrun_on_thn_seqpy)
  - [Documentation of remaining code](#documentation-of-remaining-code)

****
****

******
## Tools:

- Realization of icvStarter to execute configuration files. 

***
***

***
## Examples:
- Simple examples to show basic functions in OpenICV.

****
****

****

### Include:

- Header files of underlying functions.

****
****

****

## Source

- Source files of underlying functions.
- 
****
****

****
## ThirdParty

- CMakeLists calling for third party requirements that need installing.
****
****

****

## Sensors

- CMakeLists calling for third party requirements that need installing.
****
****

****
## Sensors

## Perception

## Planning


### visualization/run_on_seq.py:

- SSH into the paperspace server.
- $ sudo sh start_docker_image.sh
- $ cd --
- $ python deeplabv3/utils/preprocess_data.py *(ONLY NEED TO DO THIS ONCE!)*
- $ python deeplabv3/visualization/run_on_seq.py 

- - This will run the pretrained model (set on line 33 in run_on_seq.py) on all images in the Cityscapes demo sequences (stuttgart_00, stuttgart_01 and stuttgart_02) and create a visualization video for each sequence, which is saved to deeplabv3/training_logs/model_eval_seq. See [Youtube video](https://youtu.be/9e2x4dDRB-k) from the top of the page. 

****

### visualization/run_on_thn_seq.py:

- SSH into the paperspace server.
- $ sudo sh start_docker_image.sh
- $ cd --
- $ python deeplabv3/utils/preprocess_data.py *(ONLY NEED TO DO THIS ONCE!)*
- $ python deeplabv3/visualization/run_on_thn_seq.py 

- - This will run the pretrained model (set on line 31 in run_on_thn_seq.py) on all images in the Thn sequence (real-life sequence collected with a standard dash cam) and create a visualization video, which is saved to deeplabv3/training_logs/model_eval_seq_thn. See [Youtube video](https://youtu.be/9e2x4dDRB-k) from the top of the page. 


****
****

****

## Documentation of remaining code

- model/resnet.py:
- - Definition of the custom Resnet model (output stride = 8 or 16) which is the backbone of DeepLabV3.

- model/aspp.py:
- - Definition of the Atrous Spatial Pyramid Pooling (ASPP) module.

- model/deeplabv3.py:
- - Definition of the complete DeepLabV3 model.

- utils/preprocess_data.py:
- - Converts all Cityscapes label images from having Id to having trainId pixel values, and saves these to deeplabv3/data/cityscapes/meta/label_imgs. Also computes class weights according to the [ENet paper](https://arxiv.org/abs/1606.02147) and saves these to deeplabv3/data/cityscapes/meta.

- utils/utils.py:
- - Contains helper funtions which are imported and utilized in multiple files. 

- datasets.py:
- - Contains all utilized dataset definitions.
