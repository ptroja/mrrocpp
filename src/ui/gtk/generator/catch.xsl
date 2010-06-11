<?xml version="1.0" encoding="UTF-8"?>
<!--
MRROC++ GUI generator
Inc window
 -->
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<xsl:template name="catch">
<xsl:variable name="fullName" select="fullName"/>
				catch (ecp::common::ecp_robot::ECP_main_error e) {
					/* Obsluga bledow ECP */
					if (e.error_class == lib::SYSTEM_ERROR)
						fprintf(stderr, "ECP lib::SYSTEM_ERROR error in UI\n");
					/*  exit(EXIT_FAILURE);*/
				}
			
				catch (ecp::common::ecp_robot::ECP_error er) {
					/* Wylapywanie bledow generowanych przez modul transmisji danych do EDP */
					if (er.error_class == lib::SYSTEM_ERROR) { /* blad systemowy juz wyslano komunikat do SR */
						perror("ECP lib::SYSTEM_ERROR in UI");
						/* PtExit( EXIT_SUCCESS ); */
					} else {
						switch (er.error_no)
						{
							case INVALID_POSE_SPECIFICATION:
							case INVALID_COMMAND_TO_EDP:
							case EDP_ERROR:
							case INVALID_ROBOT_MODEL_TYPE:
								/* Komunikat o bledzie wysylamy do SR */
								robot_<xsl:value-of select="$fullName" />->ecp->sr_ecp_msg.message(lib::NON_FATAL_ERROR, er.error_no);
								break;
							default:
								robot_<xsl:value-of select="$fullName" />->ecp->sr_ecp_msg.message(lib::NON_FATAL_ERROR, 0, "ECP: Unidentified exception");
								perror("Unidentified exception");
						} /* end: switch */
					}
				} /* end: catch */
			
				catch (...) { /* Dla zewnetrznej petli try*/
					/* Wylapywanie niezdefiniowanych bledow*/
					/* Komunikat o bledzie wysylamy do SR (?) */
					fprintf	(stderr, "unidentified error in UI\n");
				} /*end: catch */
</xsl:template>
</xsl:stylesheet>
