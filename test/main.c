#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define PROJECT_SELECT    0

#define ACTIVE_MAIN     (PROJECT_SELECT == 0)
#define SPO_MAIN        (PROJECT_SELECT == 1)
#define DRESS_MAIN      (PROJECT_SELECT == 2)
#define VIEW_MAIN       (PROJECT_SELECT == 3)


#if ACTIVE_MAIN
# include "active_main.h"
#elif SPO_MAIN
# include "spo_main.h"
#elif DRESS_MAIN
# include "dress_main.h"
#elif VIEW_MAIN
# include "view_main.h"
#else
# error "config error!"
#endif

int main()
{

#if ACTIVE_MAIN
    active_main();
#elif SPO_MAIN
    spo_main();
#elif DRESS_MAIN
    dress_main();
#elif VIEW_MAIN
    view_main();
#endif

    return 0;
}
