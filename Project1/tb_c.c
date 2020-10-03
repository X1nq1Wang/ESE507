/*
ESE507 Projct 1 Part2 Test Value Generator
by Xinqi Wang
last modified: 10/1/2020
*/

#include <stdio.h>
#include <time.h>

int main() {
  int desiredInputs = 100;
  srand(time(NULL)); // Set random seed based on current time

  FILE *inputData, *expectedOutput;

  inputData = fopen("inputData", "w");
  expectedOutput = fopen("expectedOutput", "w");

  // To match output file
  fprintf(expectedOutput, "x\nx\n%d\n%d\n%d\n%d\n%d\n%d\n", 0, 0, 0, 0, 0, 0);
  
  //*****random test values*****
  int i, a, b, sum, valid_in;
  sum = 0;
  for (i=0; i<desiredInputs-1; i++) {
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
  
  //*****Added test cases - Over/Underflow*****
  fprintf(expectedOutput, "%d\n%d\n", 1046709, 1);
  fprintf(expectedOutput, "%d\n%d\n", 2093238, 1);
  fprintf(expectedOutput, "%d\n%d\n", 3139767, 1);
  fprintf(expectedOutput, "%d\n%d\n", 4186296, 1);
  fprintf(expectedOutput, "%d\n%d\n", 5232825, 1);
  fprintf(expectedOutput, "%d\n%d\n", 6279354, 1);
  fprintf(expectedOutput, "%d\n%d\n", 7325883, 1);
  fprintf(expectedOutput, "%d\n%d\n", 8372412, 1);
  fprintf(expectedOutput, "%d\n%d\n", -7358275, 1);
  fprintf(expectedOutput, "%d\n%d\n", 8371389, 1);
  fprintf(expectedOutput, "%d\n%d\n", 8371399, 1);

  fclose(inputData);
  fclose(expectedOutput);
}
