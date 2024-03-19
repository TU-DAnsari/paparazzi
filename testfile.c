#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// int main()
// {
//   char charactername[] = " damian";
//   int characterage = 35;
  

//   printf(" Mijn naam is %s\n", charactername);
//   printf(" ik ben %d jaar oud\n", characterage);
//   printf(" ik woon in den haag\n");
//   printf(" ik doe aerospace\n");

//   return 0;
// }

//Data types:

// int main()
// {

//   int age = 34;
//   float age = 22.2; 
//   //USE double instead of float, as it can do both int and float
//   double age = 22.2;
//   char grade = 'A'; //you can only store one character!! not more 
//   char phrase[] = " Giraffe is cool" //with this method, you can use more than one character



//   return 0;
// }

//printf
//if you printf %d, you tell c to print out a number (%f is decimal number)
// with %s you can include a text
//%c is for character, %f is float

// int main()
// {

//   printf("%f ", sqrt(6));
//   printf("%f ", floor(45.2));
//   printf("%f", ceill(6.9));
//   /*This is a comment*/


//   return 0;
// }



int main()
{

 int age;
 printf(" enter your age: ");
 scanf("%d", &age);
 printf("you are %d years old", age);

  return 0;
}
