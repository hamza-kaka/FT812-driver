/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name PORTD8 (number 137), LCD_Audiol
  @{ */
#define BOARD_INITPINS_LCD_Audiol_GPIO GPIOD /*!<@brief GPIO device name: GPIOD */
#define BOARD_INITPINS_LCD_Audiol_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_LCD_Audiol_PIN 8U     /*!<@brief PORTD pin index: 8 */
                                             /* @} */

/*! @name PORTD9 (number 138), LCD_Int
  @{ */
#define BOARD_INITPINS_LCD_Int_GPIO GPIOD /*!<@brief GPIO device name: GPIOD */
#define BOARD_INITPINS_LCD_Int_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_LCD_Int_PIN 9U     /*!<@brief PORTD pin index: 9 */
                                          /* @} */

/*! @name PORTD10 (number 139), LCD_PDN
  @{ */
#define BOARD_INITPINS_LCD_PDN_GPIO GPIOD /*!<@brief GPIO device name: GPIOD */
#define BOARD_INITPINS_LCD_PDN_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_LCD_PDN_PIN 10U    /*!<@brief PORTD pin index: 10 */
                                          /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
