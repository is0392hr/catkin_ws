
from . import pspnet
from . import mobilenet
model_from_name = {}


'''
model_from_name["vgg_pspnet"] = pspnet.vgg_pspnet
model_from_name["resnet50_pspnet"] = pspnet.resnet50_pspnet

model_from_name["vgg_pspnet"] = pspnet.vgg_pspnet
model_from_name["resnet50_pspnet"] = pspnet.resnet50_pspnet

model_from_name["pspnet_50"] = pspnet.pspnet_50
model_from_name["pspnet_101"] = pspnet.pspnet_101
'''
#model_from_name["pspnet"] = pspnet.pspnet
model_from_name["mobilenet_pspnet"] = pspnet.mobilenet_pspnet


