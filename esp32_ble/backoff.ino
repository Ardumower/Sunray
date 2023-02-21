class Backoff
{
  private:
    uint32_t min;
    uint32_t max;
    float factor;

    bool active;
    uint32_t last_time;
    uint32_t last_value;

  public:
    Backoff(uint32_t _min, uint32_t _max, float _factor);

    void reset();
    uint32_t next();
};

Backoff::Backoff(uint32_t _min, uint32_t _max, float _factor)
  : min(_min), max(_max), factor(_factor), 
    active(false), last_time(0), last_value(0)
{
}

void Backoff::reset()
{
  active = false;
  last_time = 0;
  last_value = 0;
}

uint32_t Backoff::next()
{
  if (!active)
  {
    active = true;
    last_value = min;

    return min;
  }

  uint32_t next_value = last_value * factor;
  if (next_value > max)
    next_value = max;

  last_value = next_value;

  return next_value;
}
