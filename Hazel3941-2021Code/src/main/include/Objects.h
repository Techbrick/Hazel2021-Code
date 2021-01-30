struct Ball{
    public:
        Ball(float val){
            x = val;
        }
        union{
            float theta;
            float x;
            float deg;
        };
        ~Ball(){}
};