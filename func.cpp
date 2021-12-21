#include "func.hpp"
#include <pthread.h>

using namespace std;

Mat grayThread(Mat rgb, Thread_Info *data)
{
   Mat gray = Mat(rgb.size(), CV_8UC1);
   uint8x8x3_t u8_rgb_vecs;
   uint16x8x3_t u16_rgb_vecs;
   uint16x4x3_t u16_splith;
   uint16x4x3_t u16_splitl;
   uint32x4_t u32_res_vech;
   uint32x4_t u32_res_vecl;
   uint16x4_t u16_res_vech;
   uint16x4_t u16_res_vecl;
   uint8x8_t u8_res_vec;
   uint8_t *rgb_ptr = rgb.data + (data->offset * 3);
   uint8_t *gray_ptr = gray.data + data->offset;
   int totalPixels = rgb.size().width * data->rows;
   int leftover_pixels = totalPixels % 8;
   int offset = 8 - leftover_pixels;

   uint8_t *data_end_ptr = rgb_ptr + totalPixels*3;

   //For loop that goes through and operates on 8 pixels at a time
   while (rgb_ptr < data_end_ptr - 24)
   {
      //Deinterleaves 8 pixel's RGB data
      u8_rgb_vecs = vld3_u8(rgb_ptr);

      //Ups the bits to 16 from 8
      u16_rgb_vecs.val[0] = vmovl_u8(u8_rgb_vecs.val[0]);
      u16_rgb_vecs.val[1] = vmovl_u8(u8_rgb_vecs.val[1]);
      u16_rgb_vecs.val[2] = vmovl_u8(u8_rgb_vecs.val[2]);

      //Splits the vector in half to have 4 u16
      u16_splith.val[0] = vget_high_u16(u16_rgb_vecs.val[0]);
      u16_splith.val[1] = vget_high_u16(u16_rgb_vecs.val[1]);
      u16_splith.val[2] = vget_high_u16(u16_rgb_vecs.val[2]);

      u16_splitl.val[0] = vget_low_u16(u16_rgb_vecs.val[0]);
      u16_splitl.val[1] = vget_low_u16(u16_rgb_vecs.val[1]);
      u16_splitl.val[2] = vget_low_u16(u16_rgb_vecs.val[2]);

      //Multiplies the channels with their appropriate weight (opencv orders RGB values to BGR)
      u32_res_vech = vmull_n_u16(u16_splith.val[0], B_WEIGHT);
      u32_res_vech = vmlal_n_u16(u32_res_vech, u16_splith.val[1], G_WEIGHT);
      u32_res_vech = vmlal_n_u16(u32_res_vech, u16_splith.val[2], R_WEIGHT);

      u32_res_vecl = vmull_n_u16(u16_splitl.val[0], B_WEIGHT);
      u32_res_vecl = vmlal_n_u16(u32_res_vecl, u16_splitl.val[1], G_WEIGHT);
      u32_res_vecl = vmlal_n_u16(u32_res_vecl, u16_splitl.val[2], R_WEIGHT);

      //Bit shift right to divide by 1024 and narrow the vector to 16 bits
      u16_res_vech = vshrn_n_u32(u32_res_vech, 10);
      u16_res_vecl = vshrn_n_u32(u32_res_vecl, 10);

      //Combines the two 64 bit vectors to one 128 bit register
      u16_rgb_vecs.val[0] = vcombine_u16(u16_res_vecl, u16_res_vech);
      u8_res_vec = vmovn_u16(u16_rgb_vecs.val[0]);

      vst1_u8(gray_ptr, u8_res_vec);
      rgb_ptr += 24;
      gray_ptr += 8;
   }

   //Repeat for leftover pixels
   if (leftover_pixels > 0)
   {
      rgb_ptr -= offset;
      gray_ptr -= offset;
      u8_rgb_vecs = vld3_u8(rgb_ptr);

      u16_rgb_vecs.val[0] = vmovl_u8(u8_rgb_vecs.val[0]);
      u16_rgb_vecs.val[1] = vmovl_u8(u8_rgb_vecs.val[1]);
      u16_rgb_vecs.val[2] = vmovl_u8(u8_rgb_vecs.val[2]);

      u16_splith.val[0] = vget_high_u16(u16_rgb_vecs.val[0]);
      u16_splith.val[1] = vget_high_u16(u16_rgb_vecs.val[1]);
      u16_splith.val[2] = vget_high_u16(u16_rgb_vecs.val[2]);

      u16_splitl.val[0] = vget_low_u16(u16_rgb_vecs.val[0]);
      u16_splitl.val[1] = vget_low_u16(u16_rgb_vecs.val[1]);
      u16_splitl.val[2] = vget_low_u16(u16_rgb_vecs.val[2]);

      u32_res_vech = vmull_n_u16(u16_splith.val[0], B_WEIGHT);
      u32_res_vech = vmlal_n_u16(u32_res_vech, u16_splith.val[1], G_WEIGHT);
      u32_res_vech = vmlal_n_u16(u32_res_vech, u16_splith.val[2], R_WEIGHT);

      u32_res_vecl = vmull_n_u16(u16_splitl.val[0], B_WEIGHT);
      u32_res_vecl = vmlal_n_u16(u32_res_vecl, u16_splitl.val[1], G_WEIGHT);
      u32_res_vecl = vmlal_n_u16(u32_res_vecl, u16_splitl.val[2], R_WEIGHT);

      u16_res_vech = vshrn_n_u32(u32_res_vech, 10);
      u16_res_vecl = vshrn_n_u32(u32_res_vecl, 10);
      u16_rgb_vecs.val[0] = vcombine_u16(u16_res_vecl, u16_res_vech);
      u8_res_vec = vmovn_u16(u16_rgb_vecs.val[0]);

      vst1_u8(gray_ptr, u8_res_vec);
   }
   return gray;
}

uint8x8_t sobelVector(uint8_t* TLptr, uint8_t *MLptr, uint8_t *DLptr){
   uint8x8_t topLeft = vld1_u8(TLptr);
   uint8x8_t topMid = vld1_u8(TLptr + 1);
   uint8x8_t topRight = vld1_u8(TLptr + 2);
   uint8x8_t left = vld1_u8(MLptr);
   uint8x8_t right = vld1_u8(MLptr + 2);
   uint8x8_t botLeft = vld1_u8(DLptr);
   uint8x8_t botMid = vld1_u8(DLptr + 1);
   uint8x8_t botRight = vld1_u8(DLptr + 2);

   //Convert to u16
   //Gx
   uint16x8_t topBotLeftX_u16 = vaddl_u8(topLeft, botLeft);
   uint16x8_t topBotRightX_u16 = vaddl_u8(topRight, botRight);
   uint16x8_t left_u16 = vmovl_u8(left);
   uint16x8_t right_u16 = vmovl_u8(right);

   //Gy
   uint16x8_t leftRightTopY_u16 = vaddl_u8(topLeft, topRight);
   uint16x8_t leftRightBotY_u16 = vaddl_u8(botLeft, botRight);
   uint16x8_t topMid_u16 = vmovl_u8(topMid);
   uint16x8_t botMid_u16 = vmovl_u8(botMid);

   //Convert all to signed16
   //Gx
   int16x8_t topBotLeftX_s16 = vreinterpretq_s16_u16(topBotLeftX_u16);
   int16x8_t topBotRightX_s16 =vreinterpretq_s16_u16(topBotRightX_u16);
   int16x8_t left_s16 = vreinterpretq_s16_u16(left_u16);
   int16x8_t right_s16 =vreinterpretq_s16_u16(right_u16);

   //Gy
   int16x8_t leftRightTopY_s16 = vreinterpretq_s16_u16(leftRightTopY_u16);
   int16x8_t leftRightBotY_s16 =vreinterpretq_s16_u16(leftRightBotY_u16);
   int16x8_t topMid_s16 = vreinterpretq_s16_u16(topMid_u16);
   int16x8_t botMid_s16 =vreinterpretq_s16_u16(botMid_u16);

   //Multiply by scalars
   //Gx
   topBotLeftX_s16 = vmulq_n_s16(topBotLeftX_s16, -1);   //IS IT OKAY TO STORE IN THE SAME VAR THAT IS BEING MULTIPLIED??
   left_s16 = vmulq_n_s16(left_s16, -2);
   right_s16 = vshlq_n_s16(right_s16, 1); // * 2

   //Gy
   leftRightBotY_s16 = vmulq_n_s16(leftRightBotY_s16, -1);
   topMid_s16 = vshlq_n_s16(topMid_s16, 1); // * 2
   botMid_s16 = vmulq_n_s16(botMid_s16, -2);

   //Sum and Abs val
   //Gx
   int16x8_t sumGx = vaddq_s16(topBotLeftX_s16, topBotRightX_s16);
   sumGx = vaddq_s16(sumGx, left_s16);
   sumGx = vaddq_s16(sumGx, right_s16);
   sumGx = vabsq_s16(sumGx);

   //Gy
   int16x8_t sumGy = vaddq_s16(leftRightTopY_s16, leftRightBotY_s16);
   sumGy = vaddq_s16(sumGy, topMid_s16);
   sumGy = vaddq_s16(sumGy, botMid_s16);
   sumGy = vabsq_s16(sumGy);

   //Combine Gx and Gy and convert to u8
   int16x8_t G_s16 = vaddq_s16(sumGx, sumGy);
   uint16x8_t G_u16 = vreinterpretq_u16_s16(G_s16);
   uint8x8_t G = vqmovn_u16(G_u16);

   return G;
}

void *SobelThread(Mat* m, Mat* sobelMat, Thread_Info *data) {
   int step = 8;
   int width = m->size().width;
   int leftoverPX = width % step;
   uint8_t* TLptr = m->data + data->offset;
   uint8_t* MLptr = TLptr + width;
   uint8_t* DLptr = MLptr + width;
   uint8_t* STptr = (sobelMat->data + data->offset) + 1 + width; //Store Pointer
   uint8_t* stopPtr = TLptr + width - 3;

   for (int i = 0; i < data->rows; i++){

      while (TLptr+step < stopPtr)
      {
         uint8x8_t G = sobelVector(TLptr, MLptr, DLptr);
         vst1_u8(STptr, G);

         //Update Pointers
         TLptr += step;
         MLptr += step;
         DLptr += step;
         STptr += step;
      }

      TLptr -= step - leftoverPX;
      MLptr -= step - leftoverPX;
      DLptr -= step - leftoverPX;
      STptr -= step - leftoverPX;

      uint8x8_t G = sobelVector(TLptr, MLptr, DLptr);
      vst1_u8(STptr, G);

      TLptr += 3 + step;
      MLptr += 3 + step;
      DLptr += 3 + step;
      STptr += 3 + step;
      stopPtr += width;
   }
   return NULL;
}

Buffer::Buffer()
{
   for (int i = 0; i < NUM_THREADS; i++)
      th_locat[i] = 0;
   for (int i = 0; i < BUFF_LEN; i++)
      buffer[i].stat = EMPTY_STAT;
}

int Buffer::available()
{
   if (buffer[endI].stat == EMPTY_STAT)
      return 1;
   else
      return 0;
}

//Returns Null if nothing is done
Mat Buffer::getNextDone()
{
   Mat next;
   next.data = NULL;
   
   if (buffer[getI].stat == DONE_STAT)
   {
      buffer[getI].stat = EMPTY_STAT;
      getI = (getI + 1) % BUFF_LEN;
      next = buffer[getI].m;
   }
   return next;
}

//Return -1 means the next entry is not new
int Buffer::getNext_thread(Mat** m, Mat** sobel, uint8_t id)
{
   uint8_t loc = th_locat[id];
   uint8_t mask = (1 << id);

   buffer[loc].stat = buffer[loc].stat | mask;
   loc = (loc + 1) % BUFF_LEN;
   if (buffer[loc].stat & mask)
      return -1;

   th_locat[id] = loc;
   *m = &buffer[loc].m;
   *sobel = &buffer[loc].sobelMat;
   return 0;
}

//Return -1 if buffer is full with uncompleted frames
int Buffer::fillNext(Mat m)
{
   //If Ready
   if (buffer[endI].stat == EMPTY_STAT)
   {
      buffer[endI].m = m;
      buffer[endI].sobelMat = Mat(m.size(), CV_8UC1);
      buffer[endI].stat = 0;
      endI = (endI + 1) % BUFF_LEN;
      return 0;
   }
   return -1;
} 

void *thread_main(void *threadarg)
{
   struct thread_data *data;
   data = (struct thread_data *) threadarg;
   Mat* m;
   Mat* sobelMat;
   Buffer *buf = data->buf;

   while(*(data->killFlag) == 0)
   {
      while(buf->getNext_thread(&m, &sobelMat, data->id) != -1)
      {
         *m = grayThread(*m, data);
         // SobelThread(m, sobelMat, data);
      }
      sched_yield();
   }
   pthread_exit(NULL);
   return NULL;
}