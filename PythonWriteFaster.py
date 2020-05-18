{\rtf1\ansi\ansicpg1252\cocoartf1561\cocoasubrtf600
{\fonttbl\f0\fnil\fcharset0 Menlo-Regular;}
{\colortbl;\red255\green255\blue255;\red95\green112\blue168;\red1\green25\blue64;\red255\green255\blue255;
\red200\green241\blue153;\red229\green168\blue255;\red139\green255\blue255;\red174\green208\blue255;\red254\green185\blue125;
}
{\*\expandedcolortbl;;\cssrgb\c44706\c52157\c71765;\cssrgb\c0\c14118\c31765;\cssrgb\c100000\c100000\c100000;
\cssrgb\c81961\c94510\c66275;\cssrgb\c92157\c73333\c100000;\cssrgb\c60000\c100000\c100000;\cssrgb\c73333\c85490\c100000;\cssrgb\c100000\c77255\c56078;
}
\paperw11900\paperh16840\margl1440\margr1440\vieww25400\viewh15520\viewkind0
\deftab720
\pard\pardeftab720\sl360\partightenfactor0

\f0\fs24 \cf2 \cb3 \expnd0\expndtw0\kerning0
\outl0\strokewidth0 \strokec2 # -*- coding: utf-8 -*-\cf4 \cb1 \strokec4 \
\pard\pardeftab720\sl360\partightenfactor0
\cf5 \cb3 \strokec5 """\cf4 \cb1 \strokec4 \
\cf5 \cb3 \strokec5 Created on Wed Jun 19 16:28:44 2019\cf4 \cb1 \strokec4 \
\cf5 \cb3 \strokec5 Updated on Wed May 13 14:15:33 2020\cf4 \cb1 \strokec4 \
\cf5 \cb3 \strokec5 @author: Lionel, Peng\cf4 \cb1 \strokec4 \
\cf5 \cb3 \strokec5 """\cf4 \cb1 \strokec4 \
\
\
\
\pard\pardeftab720\sl360\partightenfactor0
\cf6 \cb3 \strokec6 import\cf4 \strokec4  serial\cb1 \
\cf6 \cb3 \strokec6 import\cf4 \strokec4  pickle\cb1 \
\
\pard\pardeftab720\sl360\partightenfactor0
\cf4 \cb3 ser \cf7 \strokec7 =\cf4 \strokec4  serial.\cf8 \strokec8 Serial(\cf5 \strokec5 '/dev/cu.usbmodem1421'\cf8 \strokec8 )\cf4 \strokec4    \cf2 \strokec2 #if use serial programming port, \cf4 \cb1 \strokec4 \
\cb3                                               \cf2 \strokec2 #should add argument:baudrate = 250000                                                                                  #should add argument:baudrate = 250000\cf4 \cb1 \strokec4 \
\cb3 ser.\cf8 \strokec8 flushInput()\cf4 \cb1 \strokec4 \
\
\
\pard\pardeftab720\sl360\partightenfactor0
\cf6 \cb3 \strokec6 while\cf4 \strokec4  \cf9 \strokec9 True\cf4 \strokec4 :\cb1 \
\pard\pardeftab720\sl360\partightenfactor0
\cf4 \cb3     \cf8 \strokec8 print(\cf5 \strokec5 "Connected with:"\cf8 \strokec8 , ser.name)\cf4 \strokec4        \cf2 \strokec2 #print the connected serial port name \cf4 \cb1 \strokec4 \
\cb3     \cf6 \strokec6 try\cf4 \strokec4 :\cb1 \
\cb3         ser_bytes \cf7 \strokec7 =\cf4 \strokec4  ser.\cf8 \strokec8 read(\cf9 \strokec9 16384\cf8 \strokec8 )\cf4 \strokec4           \cf2 \strokec2 #read up to 16384 bytes \cf4 \cb1 \strokec4 \
\cb3         \cf6 \strokec6 with\cf4 \strokec4  \cf8 \strokec8 open(\cf5 \strokec5 "mypicklefile19"\cf8 \strokec8 , \cf5 \strokec5 "wb"\cf8 \strokec8 )\cf4 \strokec4  \cf6 \strokec6 as\cf4 \strokec4  f:\cb1 \
\cb3             pickle.\cf8 \strokec8 dump(ser_bytes, f)\cf4 \cb1 \strokec4 \
\cb3         \cb1 \
\cb3     \cf6 \strokec6 except\cf4 \strokec4 :\cb1 \
\cb3         \cf8 \strokec8 print(\cf5 \strokec5 "Keyboard Interrupt"\cf8 \strokec8 )\cf4 \cb1 \strokec4 \
\cb3         \cf6 \strokec6 break\cf4 \cb1 \strokec4 \
}