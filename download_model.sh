#!/bin/bash

mkdir models

export fileid='1rDOc24g7RB_1U0S7dBU1_MPZDyB629Md'
export filename='models/lstm_10_50_runsigm_runsigm.h5'

wget --save-cookies cookies.txt 'https://docs.google.com/uc?export=download&id='$fileid -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p' > confirm.txt

wget --load-cookies cookies.txt -O $filename 'https://docs.google.com/uc?export=download&id='$fileid'&confirm='$(<confirm.txt) 


rm cookies.txt confirm.txt