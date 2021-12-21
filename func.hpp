#ifndef FUNC_HPP
#define FUNC_HPP
   #include <opencv2/core.hpp>
   #include <opencv2/imgcodecs.hpp>
   #include <opencv2/highgui.hpp>
   #include <opencv2/opencv.hpp>
   #include "arm_neon.h"
   #include <map>
   using namespace cv;

   #define BUFF_LEN 1
   #define DONE_STAT 0x0F
   #define EMPTY_STAT 0xFF
   #define NUM_THREADS 4

   #define B_WEIGHT 0.0722 * 1024
   #define G_WEIGHT 0.7152 * 1024
   #define R_WEIGHT 0.2126 * 1024

   #define PIXEL(mat,y,x) (mat->at<uchar>(y,x))

   typedef struct buf_node {
      Mat m;
      Mat sobelMat;
      uint8_t stat;
   }Buf_Node;

   //Empty status is the 7th bit being set along with every bit for each thread
   class Buffer {
      private:
      std::map<uint8_t, uint8_t> th_locat;
      Buf_Node buffer[BUFF_LEN];
      uint8_t getI = 0;
      uint8_t endI = 0;

      public:
      Buffer();
      int available();
      //Returns Null if nothing is done
      Mat getNextDone();
      //Return -1 means the next entry is not new
      int getNext_thread(Mat** m, Mat** sobel, uint8_t id);
      //Return -1 if buffer is full with uncompleted frames
      int fillNext(Mat m);
   };

   typedef struct thread_data {
      int rows;
      int offset;
      int id;
      Buffer* buf;
      uint8_t* killFlag;
   }Thread_Info;

   void *thread_main(void *threadarg);
#endif

