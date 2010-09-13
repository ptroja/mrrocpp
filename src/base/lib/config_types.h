/*!
 * @file configsrv.h
 * @brief Data structures for communication with configuration server
 *
 * @author Piotr Trojanek <piotr.trojanek@gmail.com>
 *
 * @ingroup LIB
 */

#if !defined(_CONFIG_TYPES_H)
#define _CONFIG_TYPES_H

#define CONFIG_MAX_KEY_LEN				80
#define CONFIG_MAX_SECTION_NAME_LEN		80
#define CONFIG_MAX_CONFIGFILE_LEN		80

#define CONFIGSRV_CHANNEL_NAME			"configsrv"

typedef enum _config_request
{
	CONFIG_CHANGE_INI_FILE,
	CONFIG_RETURN_INT_VALUE,
	CONFIG_RETURN_DOUBLE_VALUE,
	CONFIG_RETURN_STRING_VALUE,
	CONFIG_EXISTS
} config_request_t;

typedef struct _query
{
	char key[CONFIG_MAX_KEY_LEN];
	char section[CONFIG_MAX_SECTION_NAME_LEN];
} query_t;

typedef char configfile_t[CONFIG_MAX_CONFIGFILE_LEN];

typedef union data_t
{
	query_t query;
	configfile_t configfile;
} config_msg_t;

#endif /* _CONFIG_TYPES_H */
