#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char** argv)
{
    printf("0x%s\n",argv[1]);
    unsigned long word32b = (unsigned long)strtoul(argv[1],NULL,16);
    printf("%u\n",word32b);
    printf("Begin. \n");
    for(int i = 0; i<32; i++)
    {
        printf("Bit %d : %d \n",i,((word32b>>i)&0x1));
    }
    printf("End. \n");
    return (0);
}