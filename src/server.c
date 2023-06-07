#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include "app_remap_led.h"
#define  END_SCORE 15
#define NUM_CONNECTIONS 2
extern int tempOffset; //code we added
static void linkTo(void);
void toggleLED(uint8_t);
void Print_Score();
void print_msg(char* str);

uint8_t gun1score=0;
uint8_t gun2score=0;
uint8_t gameOver=0;
int sendFlag = 1;//whom to send
int actionMode=1;//1-start,2-hit_count,3-reset,4-kill, 5-program flow
int readyCount=0;
char gun1win[] = {"The winner is gun #1\r\n"};
char gun2win[] = {"The winner is gun #2\r\n"};
static linkID_t sLinkIDs[NUM_CONNECTIONS] = {0};


/* application Rx frame handler. */
static uint8_t sRxCallback(linkID_t);

void main (void)
{
    BSP_Init();

    BCSCTL3 |= LFXT1S_2;                      // LFXT1 = VLO

    /*intial UART*/
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
    DCOCTL = CALDCO_1MHZ;
    P3SEL |= 0x30;                            // P3.4,5 = USCI_A0 TXD/RXD
    UCA0CTL1 = UCSSEL_2;                      // SMCLK
    UCA0BR0 = 104;                           // 9600 from 1Mhz
    UCA0BR1 = 0;
    UCA0MCTL = UCBRS_1;
    IE2  = 0x1;                         // enable the RX

    print_msg("                                                               _   __  __  ___ \r\n");
    print_msg("     _______________________________________________    |     /_\\ |__ |__ |__/ \r\n");
    print_msg("    /                                               |_  |___ /   \\ __||__ |  \\ \r\n");
    print_msg("   /      __________________                          |  \r\n");
    print_msg("  /                                                  _|************************\r\n");
    print_msg(" /                                                  |     _____   _     ___\r\n");
    print_msg("/____                _______________________________|       |    /_\\   |  __\r\n");
    print_msg("    /          /    /                                       |   /   \\  |___|\r\n");
    print_msg("   /          /----/  \r\n");
    print_msg("  /          /    WELCOME TO THE BEST GAME  \r\n");
    print_msg(" /          /               by\r\n");
    print_msg("/__________/      ASSAF & BARAK & TSLIL & HILA \r\n\n\n");


    SMPL_Init(sRxCallback);

    /* turn on LEDs. */
    if (!BSP_LED2_IS_ON())
    {
        toggleLED(2);
    }
    if (!BSP_LED1_IS_ON())
    {
        toggleLED(1);
    }
    /* wait for a button press... */
    print_msg("Press the MSP button\r\n");
    do {
        if (BSP_BUTTON1() || BSP_BUTTON2())
        {
            break;
        }
    } while (1);
    print_msg("Searching for GUNZZZZ to connect\r\n");
    /* never coming back... */
    linkTo();
    print_msg("\nBye Bye, Thank you for playing :)\r\n");
}

static void linkTo()
{
    uint8_t  msg[2],numConnections=0;
    int i;

    while(SMPL_SUCCESS != SMPL_Link(&sLinkIDs[numConnections])) {    //SMPL_Link()
        toggleLED(1);
        toggleLED(2);
        for(i=10000; i>0; i--);   // delay before sending
    }
    if (BSP_LED1_IS_ON())
    {
        toggleLED(1);
    }
    numConnections++;
    print_msg("1st Player connected\r\n");
    while(SMPL_SUCCESS != SMPL_Link(&sLinkIDs[numConnections])) {    //SMPL_Link()
        toggleLED(2);
        for(i=10000; i>0; i--);   // delay before sending
    }
    if (BSP_LED2_IS_ON())
    {
        toggleLED(2);
    }
    numConnections++;
    print_msg("2nd Player connected\r\n");

    TACTL = MC_0;                  // ACLK, upmode

    TACCTL0 = CCIE;                           // TACCR0 interrupt enabled
    TACCR0 = 6000;                           // ~1/2 second
    TACTL = TASSEL_1 + MC_1;                  // ACLK, upmode
    /* turn on RX. default is RX off. */
    SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);

    print_msg("***Let The Games Begin***\r\n");
    msg[0] = 1;  /* start msg */
    msg[1]=END_SCORE;
    SMPL_Send(sLinkIDs[0], msg, sizeof(msg));
    SMPL_Send(sLinkIDs[1], msg, sizeof(msg));

    actionMode=2;
    msg[0] = 2;  /* hit count msg */
    while (1)
    {
        __bis_SR_register(LPM0_bits + GIE);
        if(actionMode==2){//hit count mode
            if(gun1score<END_SCORE && gun2score<END_SCORE){//playing the game
                msg[0] = 2;  /* hit count msg */
                if(sendFlag==1){
                    SMPL_Send(sLinkIDs[0], msg, sizeof(msg));
                    sendFlag=2;
                }
                else{
                    SMPL_Send(sLinkIDs[1], msg, sizeof(msg));
                    sendFlag=1;
                }
            }
            else{
                TACTL = MC_0;                  // ACLK, upmode
                actionMode=5;
            }
        }
        if(actionMode==5){//program flow mode
            //checking who won
            if(gun1score>=END_SCORE && gun2score<END_SCORE){
                print_msg(gun1win);
                Print_Score();
            }
            else if(gun2score>=END_SCORE && gun1score<END_SCORE){
                print_msg(gun2win);
                Print_Score();
            }

            print_msg("would you like another game?\r\n press Y for yes, press N to finish:  \r\n");

            while(gameOver==0);
           // print_msg("\r\n");
            if(gameOver==1){
                msg[0] = 3;  /* reset msg */
                SMPL_Send(sLinkIDs[0], msg, sizeof(msg));
                for(i=20000;i>0;i--);
                SMPL_Send(sLinkIDs[1], msg, sizeof(msg));
                for(i=20000;i>0;i--);
                actionMode=3;
                gameOver=0;
            }
            else if(gameOver==2){
                msg[0] = 0xFF;  /* kill msg */
                SMPL_Send(sLinkIDs[0], msg, sizeof(msg));
                SMPL_Send(sLinkIDs[1], msg, sizeof(msg));
                actionMode=4;
                gameOver=0;
                break;
            }
        }
        if(actionMode==3){      //wite for 2 guns to be ready
            //if (readyCount==2)

            while(readyCount < 2);
            readyCount=0;
            msg[0] = 1;  /* start msg */
            msg[1]=END_SCORE;
            SMPL_Send(sLinkIDs[0], msg, sizeof(msg));
            SMPL_Send(sLinkIDs[1], msg, sizeof(msg));
            print_msg("\n***Another Game Starts***\r\n");
            Print_Score();
            actionMode=2;

            TACCTL0 = CCIE;                           // TACCR0 interrupt enabled
            TACCR0 = 6000;                           // ~1/2 second
            TACTL = TASSEL_1 + MC_1;                  // ACLK, upmode
        }
        if(actionMode==4){
            while(readyCount < 2);//now waiting to 2 ready to die
            break;
        }
    }
}


void toggleLED(uint8_t which)
{
    if (1 == which)
    {
        BSP_TOGGLE_LED1();
    }
    else if (2 == which)
    {
        BSP_TOGGLE_LED2();
    }
    return;
}

/* handle received frames. */
static uint8_t sRxCallback(linkID_t port)
{
    uint8_t recMsg[2], len, mode,score;
    /* is the callback for the link ID we want to handle? */
    if (port == sLinkIDs[0]) //receive message from gun 1
    {
        toggleLED(1);
        /* yes. go get the frame. we know this call will succeed. */
        if ((SMPL_SUCCESS == SMPL_Receive(sLinkIDs[0], recMsg, &len)) && len)
        {
            /* Check the application sequence number to detect
             * late or missing frames...
             */
            mode = recMsg[0];
            score=recMsg[1];

            if (mode==1)//update score
            {
                if(score!=gun2score){
                    //      toggleLED(1);
                    gun2score=score;
                    Print_Score();
                }
            }
            else if(mode==2) //ready for new game - reset
            {
                gun2score=score;
                readyCount++;
                //set game timer??
            }
            else if(mode==0xFF) //ack to kill the game
            {
                readyCount++;
                return 0;//kill??
            }
            /* drop frame. we're done with it. */
            return 1;
        }
    }
    else if (port == sLinkIDs[1])//receive message from gun 2
    {
        toggleLED(2);
        /* yes. go get the frame. we know this call will succeed. */
        if ((SMPL_SUCCESS == SMPL_Receive(sLinkIDs[1], recMsg, &len)) && len)
        {

            /* Check the application sequence number to detect
             * late or missing frames...
             */
            mode = recMsg[0];
            score = recMsg[1];

            if (mode==1)//update score
            {
                if(score!=gun1score){
                    // toggleLED(2);
                    gun1score=score;
                    Print_Score();
                }
            }
            else if(mode==2) //ready for new game - reset
            {
                gun1score=score;
                readyCount++;
                //set game timer??
            }
            else if(mode==0xFF) //ack to kill the game
            {
                readyCount++;
                return 0;//kill??
            }
            /* drop frame. we're done with it. */
            return 1;
        }
    }
    /* keep frame for later handling. */
    return 0;
}

#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
    __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {
    char recievedChar;
    if(IFG2 && UCA0RXBUF) {
        recievedChar = UCA0RXBUF;        // print the input chars into terminal
        UCA0TXBUF = UCA0RXBUF;

        if(recievedChar=='Y'||recievedChar=='y'){
            gameOver=1;
        }
        if(recievedChar=='N'||recievedChar=='n'){
            gameOver=2;
        }
        else{
            print_msg("wrong input try please try again\r\n");
        }

    }
    LPM0_EXIT;
}



void print_msg(char* str) {
    unsigned int j;
    for (j = 0; str[j] != '\0'; j++) {
        while(!(IFG2 & UCA0TXIFG)); //waiting for the buffer to be clear
        UCA0TXBUF = str[j];

    }
}

void Print_Score(){
    char sNum[6]={0,0,0,0,0,'\0'};
    if(gun1score>=END_SCORE){
        gun1score=END_SCORE;
    }
    if(gun2score>=END_SCORE){
            gun2score=END_SCORE;
    }
    sNum[0]=(int)gun1score/10+48;
    sNum[1]=(int)gun1score%10+48;
    sNum[2]=':';
    sNum[3]=(int)gun2score/10+48;
    sNum[4]=(int)gun2score%10+48;
    print_msg("The game score is: ");
    print_msg(sNum);
    print_msg("\r\n");
}

