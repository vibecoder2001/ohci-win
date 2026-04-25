#ifndef FAKE_HC_EXEC_H
#define FAKE_HC_EXEC_H

struct fake_hc;

/* Simulate one HC scheduler pass: for each ED on the Control list with
 * pending work (ED.HeadP != ED.TailP, H bit clear), pop all non-placeholder
 * TDs, mark each as successful, prepend to HCCA.DoneHead, advance
 * ED.HeadP. Set HcInterruptStatus.WDH if any TD was retired. */
void fake_hc_exec_step(struct fake_hc *hc);

#endif /* FAKE_HC_EXEC_H */
