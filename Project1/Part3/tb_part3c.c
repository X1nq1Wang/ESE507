/*
ESE507 Projct 1 Part3 Test Value Generator
by Xinqi Wang
last modified: 10/5/2020
*/

#include <stdio.h>
#include <time.h>

int main() {
  int desiredInputs = 100;
  srand(time(NULL)); // Set random seed based on current time

  FILE *inputData, *expectedOutput;

  inputData = fopen("inputData2", "w");
  expectedOutput = fopen("expectedOutput2", "w");

  // To match output file
  fprintf(expectedOutput, "x\nx\n%d\n%d\n%d\n%d\n%d\n%d\n", 0, 0, 0, 0, 0, 0);
  
  //*****random test values*****
  int i, a, b, sum, valid_in;
  sum = 0;
  for (i=0; i<desiredInputs-1; i++) {
    //a = 10; 
    //b = 10;
    a = (rand() % (1023 - (-1024) + 1)) + (-1024); 
    b = (rand() % (1023 - (-1024) + 1)) + (-1024); 
    if(i%2 == 0){
      fprintf(inputData, "%X\n%X\n%d\n", a, b, 0);
      fprintf(expectedOutput, "%d\n%d\n", sum, 0);
      }
    else{
      sum = sum + a*b;
      fprintf(inputData, "%X\n%X\n%d\n", a, b, 1);
      fprintf(expectedOutput, "%d\n%d\n", sum, 1);
      }
  }
  
  //*****reset case - output should be 0*****
  a = (rand() % (1023 - (-1024) + 1)) + (-1024); 
  b = (rand() % (1023 - (-1024) + 1)) + (-1024); 
  sum = sum + a*b;
  fprintf(inputData, "%X\n%X\n%d\n", a, b, 1);
  fprintf(expectedOutput, "%d\n%d\n", 0, 0);
  
  //*****Added test cases - Reset*****
  fprintf(expectedOutput, "%d\n%d\n", 0, 0);
  fprintf(expectedOutput, "%d\n%d\n", 0, 0);
  fprintf(expectedOutput, "%d\n%d\n", 80, 1);
  fprintf(expectedOutput, "%d\n%d\n", 180, 1);
  
  //*****Added test cases - Overflow*****
  fprintf(expectedOutput, "%d\n%d\n", 1046709, 1);
  fprintf(expectedOutput, "%d\n%d\n", 2093238, 1);
  fprintf(expectedOutput, "%d\n%d\n", 3139767, 1);
  fprintf(expectedOutput, "%d\n%d\n", 4186296, 1);
  fprintf(expectedOutput, "%d\n%d\n", 5232825, 1);
  fprintf(expectedOutput, "%d\n%d\n", 6279354, 1);
  fprintf(expectedOutput, "%d\n%d\n", 7325883, 1);
  fprintf(expectedOutput, "%d\n%d\n", 8372412, 1);
  fprintf(expectedOutput, "%d\n%d\n", 8388607, 1);
  // reset 
  fprintf(expectedOutput, "%d\n%d\n", 0, 0);
  fprintf(expectedOutput, "%d\n%d\n", 0, 0);
  //*****Underflow*****
  fprintf(expectedOutput, "%d\n%d\n", -1047552, 1);
  fprintf(expectedOutput, "%d\n%d\n", -2095104, 1);
  fprintf(expectedOutput, "%d\n%d\n", -3142656, 1);
  fprintf(expectedOutput, "%d\n%d\n", -4190208, 1);
  fprintf(expectedOutput, "%d\n%d\n", -5237760, 1);
  fprintf(expectedOutput, "%d\n%d\n", -6285312, 1);
  fprintf(expectedOutput, "%d\n%d\n", -7332864, 1);
  fprintf(expectedOutput, "%d\n%d\n", -8380416, 1);
  fprintf(expectedOutput, "%d\n%d\n", -8388608, 1);
  fprintf(expectedOutput, "%d\n%d\n", -8388608, 0);
  
  fclose(inputData);
  fclose(expectedOutput);
}
