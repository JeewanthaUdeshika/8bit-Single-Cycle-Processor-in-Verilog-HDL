loadi 4 0x0A
loadi 5 0x01
loadi 6 0x01
loadi 7 0x09
sub 4 4 5   
beq 0x01 4 6
j 0xFD              
add 1 4 7

/*
int i = 10;

while (i = 1){
    i = i - 1;
}

m = i + 9;
 */