// Small C ABI for the installed Rockchip runtime; no RKNN Python toolkit needed.
#include <rknn_api.h>
#include <cstring>
extern "C" {
int texture_init(const char* path, rknn_context* ctx, int* shape) {
  if(rknn_init(ctx,const_cast<char*>(path),0,0,nullptr)<0)return -1;
  rknn_input_output_num io={}; rknn_tensor_attr a={};
  if(rknn_query(*ctx,RKNN_QUERY_IN_OUT_NUM,&io,sizeof(io))<0 || io.n_input!=1 || io.n_output!=1 ||
     rknn_query(*ctx,RKNN_QUERY_OUTPUT_ATTR,&a,sizeof(a))<0 || a.n_dims!=4 || a.dims[0]!=1 ||
     a.n_elems>4*1024*1024 || rknn_set_core_mask(*ctx,RKNN_NPU_CORE_1)<0){rknn_destroy(*ctx);*ctx=0;return -2;}
  for(int i=0;i<4;i++)shape[i]=a.dims[i];
  return 0;
}
int texture_run(rknn_context ctx, float* input, float* output, unsigned count) {
  rknn_input in={};in.buf=input;in.size=512*512*3*sizeof(float);in.type=RKNN_TENSOR_FLOAT32;in.fmt=RKNN_TENSOR_NHWC;
  if(rknn_inputs_set(ctx,1,&in)<0 || rknn_run(ctx,nullptr)<0)return -1;
  rknn_output out={};out.want_float=1;
  if(rknn_outputs_get(ctx,1,&out,nullptr)<0)return -2;
  bool valid=out.size==count*sizeof(float);
  if(valid)std::memcpy(output,out.buf,out.size);
  rknn_outputs_release(ctx,1,&out);
  return valid?0:-3;
}
void texture_close(rknn_context ctx){if(ctx)rknn_destroy(ctx);}
}
