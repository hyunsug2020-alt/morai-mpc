"""MORAI gRPC descriptors from the official MORAI-DriveExample_GRPC SDK."""
from pathlib import Path
import grpc
from google.protobuf import descriptor_pb2, descriptor_pool, message_factory, json_format

class MoraiRPC:
    def __init__(self, address='127.0.0.1:7789'):
        self.pool = descriptor_pool.DescriptorPool()
        files = list(descriptor_pb2.FileDescriptorSet.FromString((Path(__file__).parent/'morai_api/descriptors.pb').read_bytes()).file)
        while files:
            before=len(files)
            for f in files[:]:
                try:self.pool.Add(f)
                except TypeError:continue
                files.remove(f)
            if len(files)==before:raise RuntimeError('Unresolved MORAI protobuf dependencies')
        self.channel=grpc.insecure_channel(address)
    def call(self, service, method, values=None, timeout=20):
        desc=self.pool.FindServiceByName('morai_sim_api.'+service).methods_by_name[method]
        req=message_factory.GetMessageClass(desc.input_type)()
        json_format.ParseDict(values or {},req)
        cls=message_factory.GetMessageClass(desc.output_type)
        result=self.channel.unary_unary('/morai_sim_api.'+service+'/'+method,
            request_serializer=lambda msg:msg.SerializeToString(),response_deserializer=cls.FromString)(req,timeout=timeout)
        out=json_format.MessageToDict(result,preserving_proto_field_name=True,always_print_fields_with_no_presence=True)
        if desc.output_type.full_name=='morai_sim_api.Result' and out['status']!='STATUS_CODE_SUCCESS':
            raise RuntimeError(f'{method}: {out}')
        return out
