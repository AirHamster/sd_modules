#ifndef USER_SHELL_CONF
#define USER_SHELL_CONF
/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Shell maximum input line length.
 */

#define SHELL_MAX_LINE_LENGTH       64


/**
 * @brief   Shell maximum arguments per command.
 */
#define SHELL_MAX_ARGUMENTS         4


/**
 * @brief   Shell maximum command history.
 */
#define SHELL_MAX_HIST_BUFF         8 * SHELL_MAX_LINE_LENGTH


/**
 * @brief   Enable shell command history
 */
#define SHELL_USE_HISTORY           FALSE

/**
 * @brief   Enable shell command completion
 */
#define SHELL_USE_COMPLETION        TRUE

/**
 * @brief   Shell Maximum Completions (Set to max commands with common prefix)
 */
#define SHELL_MAX_COMPLETIONS       8

/**
 * @brief   Enable shell escape sequence processing
 */
#define SHELL_USE_ESC_SEQ           TRUE

/**
 * @brief   Prompt string
 */
#define SHELL_PROMPT_STR            "ch> "

/**
 * @brief   Newline string
 */
#define SHELL_NEWLINE_STR            "\r\n"
#endif
