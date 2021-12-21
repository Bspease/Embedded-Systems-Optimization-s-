#include "func.hpp"
#include <papi.h>
#include <stdio.h>
#include <unistd.h>

#define WINDOW_NAME "Sobel Video"

void PAPICheck(int *EventSet)
{
   if (PAPI_library_init(PAPI_VER_CURRENT) != PAPI_VER_CURRENT){
      exit(1);
   }
   if (PAPI_thread_init(pthread_self) != PAPI_OK){
      exit(1);
   }
   if (sysconf(_SC_NPROCESSORS_ONLN) <= 0){
      exit(1);
   }   
   if (PAPI_create_eventset(EventSet) != PAPI_OK){
      exit(1);
   }
   if (PAPI_query_event(PAPI_L2_DCM) == PAPI_OK){
      PAPI_add_event(*EventSet, PAPI_L2_DCM);
   }
}

int main(int agrc, char** argv)
{
   pthread_t threads[NUM_THREADS];
   struct thread_data td[NUM_THREADS];
   int offset;
   int i;
   Buffer buf = Buffer();
   uint8_t doneF = 0;
   Mat frame;
   Mat gray;
   int ret = 0;
   VideoCapture cap = VideoCapture("city.mp4");
   
   long_long start_cycles, end_cycles, start_usec, end_usec, tcm;
   int frames = 0;
   int EventSet = PAPI_NULL;
   PAPICheck(&EventSet);

   start_cycles = PAPI_get_real_cyc();
   start_usec = PAPI_get_real_usec();
   if ((ret = PAPI_start(EventSet)) != PAPI_OK)
   {
      std::cout << "Papi_start Error: " << ret << std::endl;
      return -1;
   }

   for(i = 0; i < BUFF_LEN; i++)
   {
      cap.read(frame);
      if (frame.empty())
      {
         doneF = 1;
         break;
      }
      frames += 1;
      ret = buf.fillNext(frame);
   }
   if (ret == -1)
   {
      std::cerr << "BUFF FILLING Error" << std::endl;
      exit(-1);
   }

   offset = (frame.rows/NUM_THREADS) * frame.size().width;
   //Create Threads
   for(i = 0; i < NUM_THREADS; i++) {
      td[i].rows = frame.rows/NUM_THREADS; 
      td[i].offset = offset * i;
      td[i].id = i;
      td[i].buf = &buf;
      td[i].killFlag = &doneF;
      if (i == NUM_THREADS - 1)
         td[i].rows -= 1;
      ret = pthread_create(&threads[i], NULL, thread_main, &td[i]);
   
      if (ret) {
         std::cout << "Error:unable to create thread," << ret << std::endl;
         exit(-1);
      }
   }

   while(!doneF)
   {
      if (!((frame = buf.getNextDone()).empty()))
      {
         imshow(WINDOW_NAME, frame);
         if (waitKey(25) == 'q')
         {
            doneF = 1;
            break;
         }
      }
      while (buf.available())
      {
         cap.read(frame);
         if (frame.empty())
         {
            doneF = 1;
            break;
         }
         frames += 1;
         buf.fillNext(frame);
      }
   }

   //Signal Threads to terminate
   for(i=0; i < NUM_THREADS; i++)
      pthread_join(threads[i], NULL);

   while (!((frame = buf.getNextDone()).empty()))
   {
      imshow(WINDOW_NAME, frame);
      if (waitKey(25) == 'q')
         break;
   }

   end_cycles = PAPI_get_real_cyc();
   end_usec = PAPI_get_real_usec();
   if ((ret = PAPI_stop(EventSet, &tcm)) != PAPI_OK)
   {
      std::cout << "Papi_stop Error: " << ret << std::endl;
      return -1;
   }
   printf("Avg Cache Misses/Frame/Core: %lld\n", tcm/frames
      /(sysconf(_SC_NPROCESSORS_ONLN)) );
   printf("Avg Cycles/Frame/Core: %lld\n", (end_cycles - start_cycles)/frames
      /(sysconf(_SC_NPROCESSORS_ONLN)) );
   printf("Average FPS: %lld\n", frames / ((end_usec - start_usec) / 1000000));

   return 0;
}

