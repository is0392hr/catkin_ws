import numpy as np
import keras
from keras.models import *
from keras.layers import *
import keras.backend as K

from keras.layers import Conv2D, MaxPooling2D, Dropout,UpSampling2D,ZeroPadding2D,AveragePooling2D
from keras.layers.merge import Concatenate


from .config import IMAGE_ORDERING
from .model_utils import get_segmentation_model, resize_image

from .mobilenet import get_mobilenet_encoder


if IMAGE_ORDERING == 'channels_first':
    MERGE_AXIS = 1
elif IMAGE_ORDERING == 'channels_last':
    MERGE_AXIS = -1


def pool_block(feats, pool_factor):

    if IMAGE_ORDERING == 'channels_first':
        h = K.int_shape(feats)[2]
        w = K.int_shape(feats)[3]
    elif IMAGE_ORDERING == 'channels_last':
        h = K.int_shape(feats)[1]
        w = K.int_shape(feats)[2]

    pool_size = strides = [
        int(np.round(float(h) / pool_factor)),
        int(np.round(float(w) / pool_factor))]
    

    x = MaxPooling2D(pool_size, data_format=IMAGE_ORDERING,
                         strides=strides, padding='same')(feats)
    x = Conv2D(512, (1, 1), data_format=IMAGE_ORDERING,
               padding='same', use_bias=False)(x)
    x = BatchNormalization()(x)
    x = Activation('relu')(x)

    x = resize_image(x, strides, data_format=IMAGE_ORDERING)
    

    return x

#FPN-Block
def fpn(res):
    conv1 = Conv2D(64, (3, 3), activation='relu', padding='same')(res)
    conv1 = BatchNormalization()(conv1)
    conv1 = Dropout(0.2)(conv1)
    conv1 = Conv2D(64, (3, 3), activation='relu', padding='same')(conv1)
    pool1 = MaxPooling2D((2, 2))(conv1)

    conv2 = Conv2D(128, (3, 3), activation='relu', padding='same')(pool1)
    conv2 = BatchNormalization()(conv2)
    conv2 = Dropout(0.2)(conv2)
    conv2 = Conv2D(128, (3, 3), activation='relu', padding='same')(conv2)
    pool2 = MaxPooling2D((2, 2))(conv2)

    conv3 = Conv2D(256, (3, 3), activation='relu', padding='same')(pool2)
    conv3 = BatchNormalization()(conv3)
    conv3 = Dropout(0.2)(conv3)
    conv3 = Conv2D(256, (3, 3), activation='relu', padding='same')(conv3)

    up1 = Concatenate(axis=-1)([UpSampling2D((2, 2))(conv3), conv2])
    conv4 = Conv2D(128, (3, 3), activation='relu', padding='same')(up1)
    conv4 = BatchNormalization()(conv4)
    conv4 = Dropout(0.2)(conv4)
    conv4 = Conv2D(128, (3, 3), activation='relu', padding='same')(conv4)

    z2=UpSampling2D((2, 2))(conv4)
    up2 = Concatenate(axis=-1)([z2, conv1])

    conv5 = Conv2D(64, (3, 3), activation='relu', padding='same')(up2)
    conv5=BatchNormalization()(conv5)
    conv5 = Dropout(0.2)(conv5)
    conv5 = Conv2D(64, (3, 3), activation='relu', padding='same')(conv5)
    conv5 = Conv2D(32, (3, 3), activation='relu', padding='same')(conv5)

    o = Concatenate(axis=MERGE_AXIS)([UpSampling2D((2, 2))(up1),up2])
    o=Activation('sigmoid')(o)
    return o

def attention_block(a,b,st=1):
    filters=b.shape[-1]#.value
    g_conv= BatchNormalization()(a)
    g_conv= Activation('relu')(g_conv)
    g_conv= Conv2D(filters,(3,3),padding='same',strides=st)(g_conv)

    x_conv= BatchNormalization()(b)
    x_conv= Activation('relu')(x_conv)
    x_conv= Conv2D(filters,(3,3),padding='same',strides=1)(x_conv)

    gc_sum= Add()([g_conv, x_conv])

    gc_conv= BatchNormalization()(gc_sum)
    gc_conv= Activation('relu')(gc_conv)
    gc_conv= Conv2D(filters,(3,3),padding='same')(gc_conv)

    gc_mul= Multiply()([gc_conv, b])
    return gc_mul

def pre_fpn_block(a,b,n_f,st=1):
    x= attention_block(a,b,st)
    x= Concatenate(axis=-1)([x,a])

    #res_block
    t= BatchNormalization()(x)
    t= Activation('relu')(t)
    t= Conv2D(n_f,(3,3),padding='same',strides=1)(t)

    t= BatchNormalization()(t)
    t= Activation('relu')(t)
    t= Conv2D(n_f,(3,3),padding='same',strides=1)(t)

    s= Conv2D(n_f,(1,1),padding='same',strides=1)(x)
    s= BatchNormalization()(s)

    fb= Add()([t,s])
    return fb
  

def pre_fpn(f3,f4,f5):
    #F3,F4,F5  filters
    n_filters=[96,320,640]
    
    x= pre_fpn_block(f4,f5,n_filters[-1],st=1)
    f3=AveragePooling2D((2, 2))(f3)
    x= pre_fpn_block(f3,x,n_filters[0],st=1)
    
    return x


def Res_block_mini(x):
    x1=Conv2D(96, (1, 1), activation='relu', padding='same')(x)
    x2=Conv2D(48, (3, 3), activation='relu', padding='same')(x1)
    x3=Conv2D(96, (3, 3), activation='relu', padding='same')(x2)
    res=Concatenate(axis=-1)([x, x1, x2, x3])
    return res
def Transition_Layer(x):
    x=Conv2D(96, (1, 1), activation='relu', padding='same')(x)
    x=MaxPooling2D(pool_size=(1,1),strides=(1,1))(x)
    return x


def _pspnet(n_classes, encoder,  input_height=384, input_width=576):

    img_input, levels = encoder(
        input_height=input_height,  input_width=input_width)
    [f1, f2, f3, f4, f5] = levels

    o = pre_fpn(f3, f4, f5)

    for i in range(3):
        o=Res_block_mini(o)
    o=Transition_Layer(o)
    

    o = pool_block(o,3)

    o = fpn(o)

    o = Conv2D(512, (3, 3), data_format=IMAGE_ORDERING, use_bias=False)(o)
    o = BatchNormalization()(o)
    o = Activation('relu')(o)
    o = Conv2D(n_classes, (1, 1), data_format=IMAGE_ORDERING,
               padding='same')(o)
    o = resize_image(o, (8, 8), data_format=IMAGE_ORDERING)

    model = get_segmentation_model(img_input, o)
    return model


def mobilenet_pspnet( n_classes ,  input_height=224, input_width=224 ):

 	model =  _pspnet(n_classes, get_mobilenet_encoder,
                    input_height=input_height, input_width=input_width)
 	model.model_name = "mobilenet_pspnet"
 	return model


if __name__ == '__main__':
    
    m = _pspnet(101, get_mobilenet_encoder, True, 224, 224)
    
