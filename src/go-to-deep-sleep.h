void goToDeepSleep()
{
  Serial.print("Going to sleep... ");
  Serial.print(TIME_TO_SLEEP);
  Serial.println(" seconds");

  btStop();

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // Go to sleep! Zzzz
  esp_deep_sleep_start();
}
void goToDeepSleepFiveMinutes()
{
  Serial.print("Going to sleep... ");
  Serial.print("300");
  Serial.println(" sekunder");

  btStop();

  // Configure the timer to wake us up!
  esp_sleep_enable_timer_wakeup(30 * uS_TO_S_FACTOR);

  // Go to sleep! Zzzz
  esp_deep_sleep_start();
}
